#include "api.h"
#include <cmath>
#include <numbers>
#include <functional>
#include <map>
#include <set>
#define PI std::numbers::pi_v<float> // M_PI from cmath is a double, float on float operations prevent casting

class AIVisionLocalizer {
    public:
        struct MountOffsetParams {
            float x = 0.0; // lateral offsets
            float y = 0.0; // forward offsets
            float angle = 0.0; // angle offset in radians, positive is clockwise
        };
        struct Pose {
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;
            float r = 0.0;
            float theta = 0.0;
            float yaw = 0.0;
            float pitch = 0.0;
        };
        struct TagCallbacks {
            std::function<void(int id, Pose pose)> on_appear;
            std::function<void(int id)>            on_disappear;
        };
        float tag_width = 1.2606;

        AIVisionLocalizer(pros::AIVision& _VisionSensor, MountOffsetParams mount = { .x=0.0, .y=0.0, .angle=0.0 }) ;
        
        std::vector<AIVisionLocalizer::Pose> getPositionsRelative(const int tag); // Pose{dx, dy, r, theta}
        std::optional<AIVisionLocalizer::Pose> getNearestRelative(const int tag); // Pose{dx, dy, r, theta}

        std::vector<AIVisionLocalizer::Pose> getPositionsGlobal(const int tag, std::array<float, 3> robot_pose); // Pose{x, y, r, theta}
        std::optional<AIVisionLocalizer::Pose> getNearestGlobal(const int tag, std::array<float, 3> robot_pose); // Pose{x, y, r, theta}
        void registerCallback(int tag_id, TagCallbacks callbacks);
        void pollCallbacks();
    private:
        

        // AI Vision Sensor constants. These were derived from:
        // https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor
        inline static constexpr float horizontal_fov_deg = 74; 
        inline static constexpr float vertical_fov_deg = 63;
        inline static constexpr float diagonal_fov_deg = 87;
        inline static constexpr float resolution_x = 320;
        inline static constexpr float resolution_y = 240;
        // No need for these to be converted into ints they are compiled immeditely below -ZK

        // Constants created for projection
        inline static const float focal_length = (resolution_x * 0.5f) / tan(horizontal_fov_deg * PI / 360.0f); // (PI/180) / 2 = (PI/360) but less div operations. -ZK
        inline static const float half_res_x = (resolution_x * 0.5f); // multiplying by .5 is faster than dividing by 2 - ZK
        float tolerance = .9; // Detections below 90% are disregarded, as recommended from Jhon @ ATUM.
        
        // Main AI Sensor
        pros::AIVision& VisionSensor;
        MountOffsetParams mount_offset;

        //functions
        float side(float x1, float y1, float x2, float y2); // no need for this to be public -ZK
        Pose project(const pros::aivision_object_tag_s_t& tag);
        Pose applyMount(const Pose& pose);
        std::map<int, TagCallbacks> callbacks;
        std::set<int> visible_last_frame;
        pros::Task* callback_task = nullptr;
};

AIVisionLocalizer::AIVisionLocalizer(pros::AIVision& _VisionSensor, MountOffsetParams mount)
     : VisionSensor(_VisionSensor), mount_offset(mount) {
    VisionSensor.disable_detection_types(pros::AivisionModeType::all); // disable 
    VisionSensor.enable_detection_types(pros::AivisionModeType::tags);
}

/* This code applies a rotation by (x, y, theta) to the position to adjust it to the center of rotation, rather than camera location. */
AIVisionLocalizer::Pose AIVisionLocalizer::applyMount(const Pose& pose) {
    Pose adjusted = pose;
    adjusted.r = pose.r;
    const float cos_a = std::cos(mount_offset.angle);
    const float sin_a = std::sin(mount_offset.angle);

    adjusted.x = pose.x * cos_a - pose.y * sin_a + mount_offset.x;
    adjusted.y = pose.x * sin_a + pose.y * cos_a + mount_offset.y;
    adjusted.r = std::sqrtf(adjusted.x * adjusted.x + adjusted.y * adjusted.y);
    adjusted.theta = std::atan2(adjusted.x, adjusted.y);
    
    return adjusted;
}


float AIVisionLocalizer::side(float x1, float y1, float x2, float y2) { 
    float dX = x2 - x1;
    float dY = y2 - y1;
    return std::sqrtf(dX * dX + dY * dY);
}

/*

Mathematics for April Tag usage (based on 4 corners):
1. Get average point of 4 corner points
2. Get average of 4 side lengths
3. Derive focal length
    focal_length = (frame_width / 2) / tan(horizontal_fov / 2) px
4. Use Pinhole camera model to get 3D position
    y = ( tag_size * focal_length ) / apparent_size
    x = ( centroid_x - image_cx ) * y / focal_length
5. Calculate theta and r for easier use in navigation
    r = sqrt(x^2 + y^2)
    theta = atan2(x, y)
6. Calculate pitch/yaw using size ratios
    pitch_ratio = (bottom_edge - top_edge) / (bottom_edge + top_edge)
    yaw_ratio = (right_edge - left_edge) / (right_edge + left_edge)
    pitch = asin(pitch_ratio)
    yaw = asin(yaw_ratio)

*/

/* Function that projects tag location into tag location */
AIVisionLocalizer::Pose AIVisionLocalizer::project(const pros::aivision_object_tag_s_t& tag) {
    Pose testpose;
    const float average_x = (tag.x0 + tag.x1 + tag.x2 + tag.x3) / 4.0;

    const float left_edge  = side(tag.x0, tag.y0, tag.x3, tag.y3);
    const float right_edge = side(tag.x1, tag.y1, tag.x2, tag.y2);
    const float top_edge = side(tag.x0, tag.y0, tag.x1, tag.y1);
    const float bottom_edge = side(tag.x3, tag.y3, tag.x2, tag.y2);

    const float average_side = (
            left_edge + right_edge + top_edge + bottom_edge
        ) / 4.0;

    // Note that it will break down from angles past ~60deg.
    const float pitch_ratio = (bottom_edge - top_edge) / (bottom_edge + top_edge);
    const float yaw_ratio = (right_edge - left_edge) / (right_edge + left_edge);
    const float focal_length_y = (resolution_y *.5f) / std::tan(vertical_fov_deg * PI / 360.0f);
    const float half_res_y = (resolution_y * .5f);

    const float average_y = (tag.y0 + tag.y1 + tag.y2 + tag.y3) / 4.0;

    testpose.y = (tag_width * focal_length) / average_side;
    testpose.x = (average_x - half_res_x) * testpose.y / focal_length;
    testpose.z = -((average_y - half_res_y) * testpose.y / focal_length_y);

    testpose.r = std::sqrtf(testpose.x * testpose.x + testpose.y * testpose.y);
    testpose.theta = std::atan2(testpose.x, testpose.y);
    testpose.pitch = std::asin(std::clamp(pitch_ratio, -1.0f, 1.0f));
    testpose.yaw = std::asin(std::clamp(yaw_ratio, -1.0f, 1.0f));
    
    return applyMount(testpose);
}

std::vector<AIVisionLocalizer::Pose> AIVisionLocalizer::getPositionsRelative(const int tag) {
    std::vector<Pose> poses;
    const int totalcount = VisionSensor.get_object_count();
    if(totalcount < 1) return poses;
    
    for (int i = 0; i < totalcount; i++) {
        pros::AIVision::Object object = VisionSensor.get_object(i);
        if (object.id != tag || !pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) continue;
        poses.push_back(project(object.object.tag));
    }
    return poses;
}

/*
AIVisionLocalizer::get_nearest essentially derives the area of each tag (in pixels) and returns the tag with the largest area.
Logically, the tag with the largest area should be default also be the closest to the camera.
*/

//moved all to floats as precision isn't needed -ZK 4/30/26
std::optional<AIVisionLocalizer::Pose> AIVisionLocalizer::getNearestRelative(const int tag) {
    std::optional<pros::aivision_object_tag_s_t> best = std::nullopt;
    float best_area = 0;
 
    const int count = VisionSensor.get_object_count();
    if (count <= 0) return std::nullopt;
 
    for (int i = 0; i < count; i++) {
        pros::AIVision::Object object = VisionSensor.get_object(static_cast<uint32_t>(i));
 
        if (object.id != tag || !pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) continue;
 
        // Shoelace formula for quadrilateral areas
        const float area = 0.5 * std::abs(
            (object.object.tag.x0) * (object.object.tag.y1 - object.object.tag.y3) +
            (object.object.tag.x1) * (object.object.tag.y2 - object.object.tag.y0) +
            (object.object.tag.x2) * (object.object.tag.y3 - object.object.tag.y1) +
            (object.object.tag.x3) * (object.object.tag.y0 - object.object.tag.y2)
        ); //Ints are immedietly converted to floats with 0 runtime, conversionsn aren't needed. -ZK
        if (area >= best_area) {
            best_area = area;
            best = object.object.tag;
        }
    }
 
    if (!best) return std::nullopt;
    return std::optional<Pose>(project(*best));
}

/*
The following functions simply convert the output to a global position given the current robot pose {x, y, theta}
which is input from the user as an array of floats to ensure portability.
*/

std::vector<AIVisionLocalizer::Pose> AIVisionLocalizer::getPositionsGlobal(const int tag, std::array<float, 3> robot_pose) {
    std::vector<Pose> relative_poses = getPositionsRelative(tag);
    std::vector<Pose> global_poses;
    for (const Pose& relative : relative_poses) {
        Pose global = relative;
        global.theta = std::fmod(relative.theta + robot_pose[2], 2 * PI);
        global.x = robot_pose[0] + relative.r * std::cosf(global.theta);
        global.y = robot_pose[1] + relative.r * std::sinf(global.theta);
        global_poses.push_back(global);
    }
    return global_poses;
}

std::optional<AIVisionLocalizer::Pose> AIVisionLocalizer::getNearestGlobal(const int tag, std::array<float, 3> robot_pose) {
    std::optional<Pose> relative = getNearestRelative(tag);
    if (!relative) return std::nullopt;

    std::optional<Pose> global = relative;
    global->theta = std::fmod(relative->theta + robot_pose[2],2 * PI);
    global->x = robot_pose[0] + relative->r * std::cosf(global->theta);
    global->y = robot_pose[1] + relative->r * std::sinf(global->theta);
    return global;
}

/* This detects all april tags and runs designated callback functions related to it. */
void AIVisionLocalizer::pollCallbacks() {
    std::set<int> visible_this_frame;

    for (auto& [tag_id, cbs] : callbacks) {
        auto positions = getPositionsRelative(tag_id);
        if (!positions.empty()) {
            visible_this_frame.insert(tag_id);
        }
    }

    for (int id : visible_this_frame) {
        if (visible_last_frame.find(id) == visible_last_frame.end()) {
            auto nearest = getNearestRelative(id);
            if (nearest && callbacks[id].on_appear) {
                callbacks[id].on_appear(id, *nearest);
            }
        }
    }

    for (int id : visible_last_frame) {
        if (visible_this_frame.find(id) == visible_this_frame.end()) {
            if (callbacks[id].on_disappear) {
                callbacks[id].on_disappear(id);
            }
        }
    }

    visible_last_frame = visible_this_frame;
}

/* This registers a callback for a specific tag */

void AIVisionLocalizer::registerCallback(int tag_id, TagCallbacks cbs) {
    callbacks[tag_id] = cbs;

    if (callback_task == nullptr) {
        callback_task = new pros::Task([this]() {
            while (true) {
                pollCallbacks();
                pros::delay(30);
            }
        }, "AIVision Callback Task");
    }
}

#include "api.h"
#include <cmath>

class AIVisionLocalizer {
    public:
        struct Pose {
            float x = 0.0;
            float y = 0.0;
            float r = 0.0;
            float theta = 0.0;
        };
        float tag_width = 1.2606; // turns out the pixel derivation was correct
        // float tolerance_percent = 0.9; // Detections below 90% are disregarded, as recommended from Jhon @ ATUM.

        AIVisionLocalizer(pros::AIVision& _VisionSensor/*, float tolerance=0.9*/);
        
        std::vector<AIVisionLocalizer::Pose> getPositionsRelative(const int tag); // Pose{dx, dy, r, theta}
        std::optional<AIVisionLocalizer::Pose> getNearestRelative(const int tag); // Pose{dx, dy, r, theta}

        std::vector<AIVisionLocalizer::Pose> getPositionsGlobal(const int tag, std::array<float, 3> robot_pose); // Pose{x, y, r, theta}
        std::optional<AIVisionLocalizer::Pose> getNearestGlobal(const int tag, std::array<float, 3> robot_pose); // Pose{x, y, r, theta}
    private:
        // AI Vision Sensor constants. These were derived from:
        // https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor
        inline static constexpr float horizontal_fov_deg = 74;
        inline static constexpr float vertical_fov_deg = 63;
        inline static constexpr float diagonal_fov_deg = 87;
        inline static constexpr float resolution_x = 320;
        inline static constexpr float resolution_y = 240;

        // Constants created for projection
        inline static const float focal_length = (resolution_x / 2) / tan(horizontal_fov_deg * M_PI / 180.0f / 2.0f);
        inline static const float half_res_x = (resolution_x / 2);
        
        // Main AI Sensor
        pros::AIVision& VisionSensor;

        Pose project(const pros::aivision_object_tag_s_t& tag);
};

AIVisionLocalizer::AIVisionLocalizer(pros::AIVision& _VisionSensor/*, float tolerance=0.9*/)
     : VisionSensor(_VisionSensor) {
    VisionSensor.disable_detection_types(pros::AivisionModeType::all); // disable 
    VisionSensor.enable_detection_types(pros::AivisionModeType::tags);
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

*/
namespace{
    float side(float x1, float y1, float x2, float y2) {
        float dX = x2 - x1;
        float dY = y2 - y1;
        return std::sqrtf(dX * dX + dY * dY);
    }
}

AIVisionLocalizer::Pose AIVisionLocalizer::project(const pros::aivision_object_tag_s_t& tag) {
    Pose testpose;
    const float average_x = (tag.x0 + tag.x1 + tag.x2 + tag.x3) / 4.0;
    const float average_side = (
        side(tag.x0, tag.y0, tag.x1, tag.y1) +
        side(tag.x1, tag.y1, tag.x2, tag.y2) +
        side(tag.x2, tag.y2, tag.x3, tag.y3) +
        side(tag.x3, tag.y3, tag.x0, tag.y0)
    ) / 4.0;

    testpose.y = (tag_width * focal_length) / average_side;
    testpose.x = (average_x - half_res_x) * testpose.y / focal_length;
    testpose.r = std::sqrt(testpose.x * testpose.x + testpose.y * testpose.y);
    testpose.theta = std::atan2(testpose.x, testpose.y);

    return testpose;
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

std::optional<AIVisionLocalizer::Pose> AIVisionLocalizer::getNearestRelative(const int tag) {
    std::optional<pros::aivision_object_tag_s_t> best = std::nullopt;
    double best_area = 0;
 
    const int count = VisionSensor.get_object_count();
    if (count <= 0) return std::nullopt;
 
    for (int i = 0; i < count; i++) {
        pros::AIVision::Object object = VisionSensor.get_object(static_cast<uint32_t>(i));
 
        if (object.id != tag || !pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) continue;
 
        // Shoelace formula for quadrilateral areas
        const double area = 0.5 * std::abs(
            static_cast<double>(object.object.tag.x0) * (object.object.tag.y1 - object.object.tag.y3) +
            static_cast<double>(object.object.tag.x1) * (object.object.tag.y2 - object.object.tag.y0) +
            static_cast<double>(object.object.tag.x2) * (object.object.tag.y3 - object.object.tag.y1) +
            static_cast<double>(object.object.tag.x3) * (object.object.tag.y0 - object.object.tag.y2)
        );
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
        Pose global;
        global.r = relative.r;
        global.theta = std::fmod(relative.theta + robot_pose[2], 2 * M_PI);
        global.x = robot_pose[0] + relative.r * std::cos(global.theta);
        global.y = robot_pose[1] + relative.r * std::sin(global.theta);
        global_poses.push_back(global);
    }
    return global_poses;
}

std::optional<AIVisionLocalizer::Pose> AIVisionLocalizer::getNearestGlobal(const int tag, std::array<float, 3> robot_pose) {
    std::optional<Pose> relative = getNearestRelative(tag);
    if (!relative) return std::nullopt;

    Pose global;
    global.r = relative->r;
    global.theta = std::fmod(relative->theta + robot_pose[2],2 * M_PI);
    global.x = robot_pose[0] + relative->r * std::cos(global.theta);
    global.y = robot_pose[1] + relative->r * std::sin(global.theta);
    return std::optional<Pose>(global);
}

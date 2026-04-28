#include "api.h"
#include "pros/ai_vision.h"
#include <cmath>

class AIVisionLocalizer {
    public:
        struct Pose {
            float x = 0.0;
            float y = 0.0;
        };
        float tag_width;

        AIVisionLocalizer(pros::AIVision* _VisionSensor, float tag_width);
        Pose project(const pros::aivision_object_tag_s_t& tag);
        std::vector<AIVisionLocalizer::Pose> getPositions(int tag); // This may need to be moved to private
    private:
        // AI Vision Sensor constants. These were derived from:
        // https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor
        inline static constexpr int horizontal_fov_deg = 74;
        inline static constexpr int vertical_fov_deg = 63;
        inline static constexpr int diagonal_fov_deg = 87;
        inline static constexpr int resolution_x = 320;
        inline static constexpr int resolution_y = 240;

        //Constants created for projection
        inline static const float focal_length = (resolution_x / 2) / tan(horizontal_fov_deg / 2);
        inline static const float half_res_x = (resolution_x / 2);
        
        //Main AI Sensor
        pros::AIVision* VisionSensor;
};

AIVisionLocalizer::AIVisionLocalizer(pros::AIVision* _VisionSensor, float tag_width)
     : VisionSensor(_VisionSensor), tag_width(tag_width) {
    VisionSensor->disable_detection_types(pros::AivisionModeType::all); // disable 
    VisionSensor->enable_detection_types(pros::AivisionModeType::tags);
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

float side(float x1, float y1, float x2, float y2) {
    float dX = x2 - x1;
    float dY = y2 - y1;
    return std::sqrtf(dX * dX + dY * dY);
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

    return testpose;//math to project tag location to coordinates, do later
}

std::vector<AIVisionLocalizer::Pose> AIVisionLocalizer::getPositions(const int tag) {
    std::vector<Pose> poses;
    const int totalcount = VisionSensor->get_object_count();
    if(totalcount < 1) return poses;

    for (int i = 0; i < totalcount; i++) {
        pros::AIVision::Object object = VisionSensor->get_object(i);
        if (object.id != tag || pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) continue;
        poses.push_back(project(object.object.tag));
    }
    return poses;
}
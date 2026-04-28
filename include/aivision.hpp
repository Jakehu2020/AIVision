#include "api.h"
#include "pros/ai_vision.h"
#include <cmath>

class AIVision {
    public:
        struct Pose {
            float x = 0.0;
            float y = 0.0;
        };
        pros::AIVision sensor;
        float tag_width;

        // AI Vision Sensor constants. These were derived from:
        // https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor
        const int horizontal_fov_deg = 74;
        const int vertical_fov_deg = 63;
        const int diagonal_fov_deg = 87;
        const int resolution_x = 320;
        const int resolution_y = 240;

        AIVision(int port, float tag_width);
        Pose project(pros::aivision_object_tag_s_t& tag);
        std::vector<AIVision::Pose> getPositions(int tag);
};

AIVision::AIVision(int port, float tag_width)
     : sensor(pros::AIVision(port)), tag_width(tag_width) {
    sensor.enable_detection_types(pros::AivisionModeType::tags);
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
    return std::sqrt(dX * dX + dY * dY);
}

AIVision::Pose AIVision::project(pros::aivision_object_tag_s_t& tag) {
    Pose testpose;
    const float average_x = (tag.x0 + tag.x1 + tag.x2 + tag.x3) / 4.0;
    const float average_side = (
        side(tag.x0, tag.y0, tag.x1, tag.y1) +
        side(tag.x1, tag.y1, tag.x2, tag.y2) +
        side(tag.x2, tag.y2, tag.x3, tag.y3) +
        side(tag.x3, tag.y3, tag.x0, tag.y0)
    ) / 4.0;

    double focal_length = (resolution_x / 2) / tan(horizontal_fov_deg / 2);
    
    testpose.y = (tag_width * focal_length) / average_side;
    testpose.x = (average_x - resolution_x/2) * testpose.y / focal_length;

    return testpose;//math to project tag location to coordinates, do later
}

std::vector<AIVision::Pose> AIVision::getPositions(const int tag) {
    std::vector<Pose> poses;
    const int totalcount = sensor.get_object_count();
    if(totalcount < 1) return poses;

    for (int i = 0; i < totalcount; i++) {
        pros::AIVision::Object object = sensor.get_object(i);
        if (object.id != tag || pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) continue;
        poses.push_back(project(object.object.tag));
    }
    return poses;
}

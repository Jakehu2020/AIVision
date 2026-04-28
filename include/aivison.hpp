#include "api.h"
#include "pros/ai_vision.h"
#include <cmath>


class AIVision {
    public:
        struct Pose {
            double x = 0.0;
            double y = 0.0;
        };
        pros::AIVision sensor;
        double tag_width;
        double fov;
        int resolution;


        AIVision(int port, double tag_width, double fov, int resolution);
        Pose project(pros::aivision_object_tag_s_t& tag);
        std::vector<AIVision::Pose> getPositions(int tag);
};


AIVision::AIVision(int port, double tag_width, double fov, int resolution)
     : sensor(pros::AIVision(port)), tag_width(tag_width), fov(fov), resolution(resolution) {
    sensor.enable_detection_types(pros::AivisionModeType::tags);
}


AIVision::Pose AIVision::project(pros::aivision_object_tag_s_t& tag) {
    Pose testpose;
    return testpose;//math to project tag location to coordinates, do later
}


std::vector<AIVision::Pose> AIVision::getPositions(int tag) {
    std::vector<Pose> poses;
    int totalcount = sensor.get_object_count();
    if(totalcount < 1) return poses;


    for (int i = 0; i < totalcount; i++) {
        pros::AIVision::Object object = sensor.get_object(i);
        if (object.id != tag || pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) continue;
        poses.push_back(project(object.object.tag));
    }
    return poses;
}


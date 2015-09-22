//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_CAMERAMODEL_HPP
#define QUADTARGETFSM_CAMERAMODEL_HPP

namespace targetfinder {

    class CameraModel {
        friend class TargetFinder;
    public:
        static constexpr double FOCAL_LENGTH = 3.6;
        static constexpr double SENSOR_WIDTH = 3.67;
        static constexpr double SENSOR_HEIGHT = 2.74;
        static constexpr double MAGNIFICATION_FACTOR = 1.0;
        static constexpr double TARGET_WIDTH = 594;
        static constexpr double TARGET_HEIGHT = 420;

        double distance(int calc_width, int calc_height);
        CameraModel(int image_width, int image_height, double target_width=TARGET_WIDTH,
                    double target_height=TARGET_HEIGHT, double focal_length=FOCAL_LENGTH,
                    double magnification_factor=MAGNIFICATION_FACTOR, double sensor_width=SENSOR_WIDTH,
                    double sensor_height=SENSOR_HEIGHT);
    protected:
        int image_width, image_height, image_size;
        double target_width = TARGET_WIDTH, target_height = TARGET_HEIGHT;
        double focal_length = FOCAL_LENGTH, magnification_factor = MAGNIFICATION_FACTOR;
        double sensor_width = SENSOR_WIDTH, sensor_height = SENSOR_HEIGHT;
        double sensor_size, real_focal_length, target_size;
    };

}

#endif //QUADTARGETFSM_CAMERAMODEL_HPP

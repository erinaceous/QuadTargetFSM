//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_CAMERAMODEL_HPP
#define QUADTARGETFSM_CAMERAMODEL_HPP

namespace targetfinder {

    class CameraModel {
        friend class TargetFinder;
    public:
        static constexpr float FOCAL_LENGTH = 3.6;
        static constexpr float SENSOR_WIDTH = 3.67;
        static constexpr float SENSOR_HEIGHT = 2.74;
        static constexpr float MAGNIFICATION_FACTOR = 1.0;
        static constexpr float TARGET_WIDTH = 594;
        static constexpr float TARGET_HEIGHT = 420;

        float distance(int calc_width, int calc_height);
        CameraModel(int image_width, int image_height, float target_width=TARGET_WIDTH,
                    float target_height=TARGET_HEIGHT, float focal_length=FOCAL_LENGTH,
                    float magnification_factor=MAGNIFICATION_FACTOR, float sensor_width=SENSOR_WIDTH,
                    float sensor_height=SENSOR_HEIGHT);
    protected:
        int image_width, image_height;
        float target_width = TARGET_WIDTH, target_height = TARGET_HEIGHT;
        float focal_length = FOCAL_LENGTH, magnification_factor = MAGNIFICATION_FACTOR;
        float sensor_width = SENSOR_WIDTH, sensor_height = SENSOR_HEIGHT;
    };

}

#endif //QUADTARGETFSM_CAMERAMODEL_HPP

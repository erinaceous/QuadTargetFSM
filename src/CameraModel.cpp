//
// Created by owain on 06/08/15.
//

#include "utils/Utils.h"
#include "include/CameraModel.hpp"

using namespace targetfinder;

CameraModel::CameraModel(int image_width, int image_height, double target_width, double target_height, double focal_length,
                         double magnification_factor, double sensor_width, double sensor_height) {
    this->image_width = image_width;
    this->image_height = image_height;
    this->image_size = MAX(image_width, image_height);
    this->target_width = target_width;
    this->target_height = target_height;
    this->target_size = MAX(target_width, target_height);
    this->focal_length = focal_length;
    this->magnification_factor = magnification_factor;
    this->real_focal_length = focal_length * magnification_factor;
    this->sensor_width = sensor_width;
    this->sensor_height = sensor_height;
    this->sensor_size = MAX(sensor_width, sensor_height);
}

double CameraModel::distance(int calc_width, int calc_height) {
    int calc_size = MAX(calc_width, calc_height);
    return (
            (this->real_focal_length * this->target_size * this->image_size) /
            (calc_size * this->sensor_size)
    );
}

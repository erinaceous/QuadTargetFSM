//
// Created by owain on 06/08/15.
//

#include "TargetFinder.h"

using namespace targetfinder;

CameraModel::CameraModel(int image_width, int image_height, double target_width, double target_height, double focal_length,
                         double magnification_factor, double sensor_width, double sensor_height) {
    this->image_width = image_width;
    this->image_height = image_height;
    this->target_width = target_width;
    this->target_height = target_height;
    this->focal_length = focal_length;
    this->magnification_factor = magnification_factor;
    this->sensor_width = sensor_width;
    this->sensor_height = sensor_height;
}

double CameraModel::distance(int calc_width, int calc_height) {
    int calc_size = MAX(calc_width, calc_height);
    int image_size = MAX(this->image_width, this->image_height);
    double target_size = MAX(this->target_width, this->target_height);
    double sensor_size = MAX(this->sensor_width, this->sensor_height);
    double focal_length = this->focal_length * this->magnification_factor;
    return (focal_length * target_size * image_size) / (calc_size * sensor_size);
}
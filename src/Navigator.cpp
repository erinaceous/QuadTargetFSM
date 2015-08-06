//
// Created by owain on 06/08/15.
//

#include <opencv2/opencv.hpp>
#include "Utils.c"
#include "TargetFinder.h"

using namespace targetfinder;

Navigator::Navigator(int width, int height, float rotation_deadzone, float horizontal_deadzone,
                     float vertical_deadzone, float update_rate) {
    this->width = width;
    this->height = height;
    this->image_center = cv::Point(width / 2, height / 2);
    this->rotation_deadzone = rotation_deadzone;
    this->horizontal_deadzone = horizontal_deadzone;
    this->vertical_deadzone = vertical_deadzone;
    this->alpha = update_rate;
    this->x = 0.5, this->y = 0.5;
}

bool Navigator::update(cv::Point target, float angle, float distance) {
    float target_x = target.x / (float) this->width;
    float target_y = target.y / (float) this->height;
    if(target_x < (0.5 - this->horizontal_deadzone)
       || target_x > (0.5 + this->horizontal_deadzone)) {
        this->x += (target_x - 0.5) * this->alpha;
    } else {
        this->x += (0.5 - this->x) * this->alpha;
    }
    if(target_y < (0.5 - this->vertical_deadzone)
       || target_y > (0.5 + this->vertical_deadzone)) {
        this->y += (target_y - 0.5) * this->alpha;
    } else {
        this->y += (0.5 - this->y) * this->alpha;
    }
    if(angle < -this->rotation_deadzone || angle > this->rotation_deadzone) {
        if(angle > 0 && angle <= 180) {
            this->angle = 45;
        } else if(angle >= -180 && angle < 0) {
            this->angle = -45;
        }
    } else {
        this->angle = 0;
    }
    if(this->x < 0) this->x = 0;
    if(this->x > 1) this->x = 1;
    if(this->y < 0) this->y = 0;
    if(this->y > 1) this->y = 1;
}

void Navigator::update() {
    this->x += (0.5 - this->x) * this->alpha;
    this->y += (0.5 - this->y) * this->alpha;
    this->angle = 0;
}

float Navigator::horizontal() {
    return this->x;
}

float Navigator::vertical() {
    return this->y;
}

float Navigator::rotation() {
    return this->angle;
}

cv::Point Navigator::image_point(int axis, int length) {
    switch(axis) {
        case 0:
            return cv::Point(
                    (this->image_center.x - length) + (this->x * (2 * length)), this->image_center.y
            );
        case 1:
            return cv::Point(
                    this->image_center.x, (this->image_center.y - length) + (this->y * (2 * length))
            );
        case 2:
            return cv::Point(
                    (this->image_center.x - length) + (this->x * (2 * length)),
                    (this->image_center.y - length) + (this->y * (2 * length))
            );
    }
}
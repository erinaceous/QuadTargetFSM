//
// Created by owain on 03/08/15.
//

#include <opencv2/opencv.hpp>
#include "Utils.c"
#include "TargetFinder.h"

using namespace targetfinder;

void PersistentTarget::update(Target *new_target, int64 timestamp, double influence) {
    cv::Rect other_rect = new_target->rect();
    double other_angle = new_target->angle();
    if(!this->alive(timestamp)) {
        this->x = other_rect.x;
        this->y = other_rect.y;
        this->width = other_rect.width;
        this->height = other_rect.height;
        this->calc_angle = other_angle;
        this->lifetime = 0;
    } else {
        this->x -= (this->x - other_rect.x) * influence;
        this->y -= (this->y - other_rect.y) * influence;
        this->width -= (this->width - other_rect.width) * influence;
        this->height -= (this->height - other_rect.height) * influence;
        this->calc_angle -= (this->calc_angle - other_angle) * influence;
    }
    this->last_updated = timestamp;
    this->lifetime += 1;
}

bool PersistentTarget::alive(int64 timestamp) {
    return (this->last_updated + this->timeout_ticks) >= timestamp;
}

double PersistentTarget::angle() {
    return this->calc_angle;
}

cv::Rect PersistentTarget::rect() {
    return cv::Rect(
            this->x, this->y, this->width, this->height
    );
}

cv::Point PersistentTarget::center() {
    return cv::Point(
            this->x + (this->width / 2),
            this->y + (this->height / 2)
    );
}

void PersistentTarget::setTimeout(int64 ticks) {
    this->timeout_ticks = ticks;
}

void PersistentTarget::setAngleOffset(double angle) {
    this->angle_offset = angle;
}

int PersistentTarget::age() {
    return this->lifetime;
}

std::string PersistentTarget::str() {
    std::stringstream ss;
    ss << "\"x\": " << this->x << ", \"y\": " << this->y;
    ss << ", \"w\": " << this->width << ", \"h\": " << this->height;
    ss << ", \"angle\": " << _degrees(this->calc_angle + this->angle_offset) << ", \"updated\": " << this->last_updated;
    ss << ", \"age\": " << this->lifetime;
    return ss.str();
}
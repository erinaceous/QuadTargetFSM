//
// Created by owain on 06/08/15.
//

#include <math.h>
#include <opencv2/opencv.hpp>
#include "include/Navigator.hpp"

using namespace targetfinder;

Navigator::PID::PID(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->integral = 0.0;
    this->previous_error = 0.0;
}

void Navigator::PID::reset(float val) {
    this->integral = 0.0;
    this->previous_error = 0.0;
}

float Navigator::PID::step(float setpoint, float value, float deltatime) {
    float error = setpoint - value;
    this->integral += error * deltatime;
    float derivative = (error - this->previous_error) / deltatime;
    float output = (this->Kp * error)
                   + (this->Ki * this->integral)
                   + (this->Kd * derivative);
    this->previous_error = error;
    return output;
}

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
    this->pitch_pid = new PID();
    this->roll_pid = new PID();
    this->yaw_pid = new PID();
    this->throttle_pid = new PID();
}

bool Navigator::update(cv::Point target, float angle, float distance, int age, float deltatime) {
    float scale = ((float) age * 0.1) * this->alpha;
    float target_x = target.x / (float) this->width;
    float target_y = target.y / (float) this->height;

    /*if(target_x < (0.5 - this->horizontal_deadzone)
       || target_x > (0.5 + this->horizontal_deadzone)) {
        // this->x += (target_x - 0.5) * scale;
        this->x = this->roll_pid->step(target_x, 0.5, deltatime);
    } else {
        this->x = this->roll_pid->step(0.5, this->x, deltatime);
    }
    if(target_y < (0.5 - this->vertical_deadzone)
       || target_y > (0.5 + this->vertical_deadzone)) {
        this->y = this->pitch_pid->step(target_y, 0.5, deltatime);
    } else {
        this->y = this->pitch_pid->step(0.5, this->y, deltatime);
    }
    if(angle < -this->rotation_deadzone || angle > this->rotation_deadzone) {
        if(angle > 0 && angle <= 180) {
            this->angle = 45;
        } else if(angle >= -180 && angle < 0) {
            this->angle = -45;
        }
    } else {
        this->angle = 0;
    }*/
    this->x = this->roll_pid->step(target_x, 0.5, deltatime) + 0.5;
    this->y = this->pitch_pid->step(target_y, 0.5, deltatime) + 0.5;
    this->angle = this->yaw_pid->step(M_PI_2, angle, deltatime) + M_PI_2;
}

void Navigator::update(float deltatime) {
    // this->x = 0.5;
    // this->y = 0.5;
    this->angle = M_PI_2;
    this->x += (0.5 - this->x) * (deltatime * this->alpha);
    this->y += (0.5 - this->y) * (deltatime * this->alpha);
    this->pitch_pid->reset(0.5);
    this->roll_pid->reset(0.5);
    // this->roll_pid->reset(0.5);
    // this->pitch_pid->reset(0.5);
    this->roll_pid->step(0.5, this->x, deltatime);
    this->pitch_pid->step(0.5, this->y, deltatime);
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

void Navigator::setPIDs(float pitch_P, float pitch_I, float pitch_D,
                        float roll_P, float roll_I, float roll_D,
                        float yaw_P, float yaw_I, float yaw_D,
                        float throt_P, float throt_I, float throt_D) {
    this->pitch_pid->Kp = pitch_P;
    this->pitch_pid->Ki = pitch_I;
    this->pitch_pid->Kd = pitch_D;
    this->roll_pid->Kp = roll_P;
    this->roll_pid->Ki = roll_I;
    this->roll_pid->Kd = roll_D;
    this->yaw_pid->Kp = yaw_P;
    this->yaw_pid->Ki = yaw_I;
    this->yaw_pid->Kd = yaw_D;
    this->throttle_pid->Kp = throt_P;
    this->throttle_pid->Ki = throt_I;
    this->throttle_pid->Kd = throt_D;
}

std::string Navigator::str() {
    std::stringstream ss;
    ss << "\"roll\": " << this->x << ", \"pitch\": " << this->y
       << ", \"yaw\": " << this->angle << ", \"throt\": " << this->throttle;
    return ss.str();
}

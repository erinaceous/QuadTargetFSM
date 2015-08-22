//
// Created by owain on 02/08/15.
//

#include "include/Marker.hpp"
#include "include/StateMachine.hpp"

using namespace targetfinder;

StateMachine::StateMachine(int input_length, double tolerance, int min_pixels) {
    this->input_length = input_length;
    this->tolerance = tolerance;
    this->min_pixels = min_pixels;
    this->max_pixels = (int)((double)input_length / ((double)StateMachine::NUM_STATES - 2));
    this->min_bound = 0;
    this->max_bound = 0;
}

void StateMachine::setInputLength(int input_length) {
    this->input_length = input_length;
}

bool StateMachine::in_bounds(int count, int scale) {
    return count >= (this->min_bound * scale) && count <= (this->max_bound * scale)
            && count <= (this->max_pixels * scale);
}
#define IN_BOUNDS(x) this->in_bounds(this->pixel_counts[x])
#define IN_BOUNDS3(x) this->in_bounds(this->pixel_counts[x], 3)

void StateMachine::reset(int y, bool value1, bool value2) {
    this->last_value1 = value1;
    this->last_value2 = value2;
    this->y = y;
    this->first_x = 0;
    this->last_x = 0;
    for(int i=0; i<StateMachine::NUM_STATES; i++) {
        this->pixel_counts[i] = 0;
    }
    this->state = 0;
    this->min_bound = 0;
    this->max_bound = 0;
}

bool StateMachine::valid_transition(int x, bool value1, bool value2) {
    switch(this->state) {
        case 0:
            return (!value1 && x != 0 && this->last_value2);
        case 1:
            this->min_bound = (int) (((double)this->pixel_counts[1] * this->tolerance));
            this->max_bound = (int) (((double)this->pixel_counts[1] / this->tolerance));
            return (value1 && this->pixel_counts[1] >= this->min_pixels
                    && this->pixel_counts[1] <= this->max_pixels);
        case 2:
        case 4:
            return (!value1 && IN_BOUNDS(this->state));
        case 3:
            return (value1 && IN_BOUNDS3(3));
        case 5:
            return ((value1 && IN_BOUNDS(5)) || x >= this->input_length - 1);
        default:
            return false;
    }
}

void StateMachine::transition(int x, int y, bool value1, bool value2) {
    if(this->valid_transition(x, value1, value2)) {
        this->state = (this->state + 1) % StateMachine::NUM_STATES;
    } else {
        this->reset(y, value1, value2);
    }
}

Marker* StateMachine::step(int x, int y, bool value1, bool value2) {
    Marker *m = nullptr;
    if(value1 != this->last_value1
       || (this->state == 0 && (value1 != this->last_value1 && value2 != this->last_value2))
       || this->state == 6) {
        if(this->state == 0) {
            this->first_x = x;
            this->last_x = x;
        }
        if(this->state == 5) {
            this->last_x = x;
        }
        if(this->state == 6) {
            m = new Marker(
                    this->first_x, this->last_x, this->y, this->y,
                    this->pixel_counts[1], this->pixel_counts[2],
                    this->pixel_counts[3]
            );
        }
        this->transition(x, y, value1, value2);
    }
    this->pixel_counts[this->state]++;
    if(this->state == 1) {
        if(this->pixel_counts[1] > this->max_pixels) {
            this->reset(y, value1);
        }
    } else if(this->state > 1) {
        int max_bound = this->max_bound;
        if(this->state == 3) {
            max_bound *= 3;
        }
        if(this->pixel_counts[this->state] > max_bound || this->pixel_counts[this->state] > this->max_pixels) {
            this->reset(y, value1);
        }
    }
    this->y = y;
    this->last_value1 = value1;
    this->last_value2 = value2;
    return m;
}

cv::Vec3b StateMachine::state_colour() {
    switch(this->state) {
        case 0:
            return cv::Vec3b(128, 128, 128);
        case 1:
            return cv::Vec3b(0, 0, 255);
        case 2:
            return cv::Vec3b(0, 128, 255);
        case 3:
            return cv::Vec3b(0, 255, 255);
        case 4:
            return cv::Vec3b(0, 255, 0);
        case 5:
            return cv::Vec3b(255, 128, 0);
        case 6:
            return cv::Vec3b(255, 0, 255);
        default:
            return cv::Vec3b(64, 64, 64);
    }
}

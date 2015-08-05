//
// Created by owain on 02/08/15.
//

#include "TargetFinder.h"
#include <math.h>

using namespace targetfinder;

Marker::Marker(int start_x, int end_x, int start_y, int end_y, int black_count, int white_count, int center_count) {
    this->start_x = start_x;
    this->start_y = start_y;
    this->end_x = end_x;
    this->end_y = end_y;
    this->black_count = black_count;
    this->white_count = white_count;
    this->center_count = center_count;
}

std::shared_ptr<Marker> Marker::cloned_shared_ptr() {
    return std::make_shared<Marker>(this->start_x, this->end_x, this->start_y, this->end_y,
                                    this->black_count, this->white_count, this->center_count);
}

bool Marker::contains(Marker other) {
    /* return (
            other.start_y >= this->start_y &&
            other.start_x >= this->start_x &&
            other.end_y <= this->end_y &&
            other.end_x <= this->end_x
    ); */
    return this->rect().contains(other.center());
}

int Marker::xlength() {
    return this->end_x - this->start_x;
}

int Marker::ylength() {
    return this->end_y - this->start_y;
}

cv::Point Marker::center() {
    return cv::Point(
            this->start_x + ((this->end_x - this->start_x) / 2),
            this->start_y + ((this->end_y - this->start_y) / 2)
    );
}

cv::Point2f Marker::centerf() {
    return cv::Point2f(
            (double)this->start_x + ((this->end_x - this->start_x) / 2.0),
            (double)this->start_y + ((this->end_y - this->start_y) / 2.0)
    );
}

double Marker::distance(Marker one, Marker two) {
    cv::Point a = one.center();
    cv::Point b = two.center();
    return sqrt(
            pow(b.x - a.x, 2) + pow(b.y - a.y, 2)
    );
}

double Marker::angle(Marker one, Marker two) {
    cv::Point a = one.center();
    cv::Point b = two.center();
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    if(dx == 0) {
        return 0;
    }
    return atan2(dy, dx);
}

void Marker::expand(Marker other, bool rotate) {
    int o_start_x, o_start_y, o_end_x, o_end_y;
    if(rotate) {
        o_start_x = other.start_y;
        o_start_y = other.start_x;
        o_end_x = other.end_y;
        o_end_y = other.end_x;
    } else {
        o_start_x = other.start_x;
        o_start_y = other.start_y;
        o_end_x = other.end_x;
        o_end_y = other.end_y;
    }
    if(o_start_x < this->start_x) {
        this->start_x = o_start_x;
    }
    if(o_start_y < this->start_y) {
        this->start_y = o_start_y;
    }
    if(o_end_x > this->end_x) {
        this->end_x = o_end_x;
    }
    if(o_end_y > this->end_y) {
        this->end_y = o_end_y;
    }
}

cv::Rect Marker::rect() {
    return cv::Rect(
          this->start_x, this->start_y,
          this->xlength(), this->ylength()
    );
}
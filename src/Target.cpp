//
// Created by owain on 02/08/15.
//

#include <memory>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "utils/Utils.h"
#include "include/Marker.hpp"
#include "include/Target.hpp"

using namespace targetfinder;

bool Target::addMarker(std::shared_ptr<Marker> m) {
    if(this->marker_count > 2) {
        return false;
    }
    this->markers[this->marker_count++] = m;
    return true;
}

bool Target::valid() {
    return this->marker_count == 3;
}

cv::Rect Target::rect() {
    cv::Rect rect = this->markers[0]->rect();
    int top = rect.y, left = rect.x;
    int bottom = rect.y + rect.height, right = rect.x + rect.width;
    for(int i=1; i<this->marker_count; i++) {
        cv::Rect new_rect = this->markers[i]->rect();
        cv::Point new_rect_br = new_rect.br();
        if(new_rect.x < left) {
            left = new_rect.x;
        }
        if((new_rect_br.x) > right) {
            right = new_rect_br.x;
        }
        if(new_rect.y < top) {
            top = new_rect.y;
        }
        if(new_rect_br.y > bottom) {
            bottom = new_rect_br.y;
        }

    }
    return cv::Rect(
            left, top, (right - left), (bottom - top)
    );
}

cv::RotatedRect Target::rotatedRect() {
    cv::Rect rect = this->rect();
    return cv::RotatedRect(
            this->center(), cv::Size2f(rect.width, rect.height), (this->calc_angle * 180 / M_PI)
    );
}

cv::Point Target::center() {
    cv::Rect rect = this->rect();
    return cv::Point(
            rect.x + (rect.width / 2),
            rect.y + (rect.height / 2)
    );
}

double Target::angle() {
    return this->calc_angle;
}

void Target::calcGeometry() {
    double a01 = Marker::angle(*this->markers[0], *this->markers[1]);
    double a02 = Marker::angle(*this->markers[0], *this->markers[2]);
    double a10 = Marker::angle(*this->markers[1], *this->markers[0]);
    double a12 = Marker::angle(*this->markers[1], *this->markers[2]);
    double a20 = Marker::angle(*this->markers[2], *this->markers[0]);
    double a21 = Marker::angle(*this->markers[2], *this->markers[1]);
    double a0 = abs(a01 - a02);
    double a1 = abs(a10 - a12);
    double a2 = abs(a20 - a21);

    if(a0 > M_PI) a0 = M_PI - a0;
    if(a1 > M_PI) a1 = M_PI - a0;
    if(a2 > M_PI) a2 = M_PI - a0;

    // cv::Point center_one, center_two;

    if(a0 >= a1 && a0 >= a2) {
        this->corner = markers[0];
        this->corner_angle = a0;
        // center_one = markers[1]->center();
        // center_two = markers[2]->center();
    }

    if(a1 >= a0 && a1 >= a2) {
        this->corner = markers[1];
        this->corner_angle = a1;
        // center_one = markers[0]->center();
        // center_two = markers[2]->center();
    }

    if(a2 >= a0 && a2 >= a1) {
        this->corner = markers[2];
        this->corner_angle = a2;
        // center_one = markers[0]->center();
        // center_two = markers[1]->center();
    }

    /* cv::Point new_center = cv::Point(
            (center_one.x + center_two.x) / 2,
            (center_one.y + center_two.y) / 2
    ); */
    cv::Point new_center = this->center();
    cv::Point corner_center = this->corner->center();
    double dx = corner_center.x - new_center.x;
    double dy = corner_center.y - new_center.y;
    this->calc_angle = atan2(dy, dx);
    this->calc_length = sqrt((dx * dx) + (dy * dy));
}

bool Target::isClose(std::shared_ptr<Marker> m) {
    if(this->marker_count > 2) {
        return false;
    }
    double avg_size, avg_width;
    for(int i=0; i<this->marker_count; i++) {
        avg_size += this->markers[i]->rect().area();
        avg_width += this->markers[i]->xlength();
    }
    avg_size /= (double) this->marker_count;
    avg_width /= (double) this->marker_count;
    double min_size = avg_size * this->marker_size_tolerance;
    double max_size = avg_size / this->marker_size_tolerance;
    double marker_area = m->rect().area();
    if(marker_area >= min_size && marker_area <= max_size) {
        for(int i=0; i<this->marker_count; i++) {
            double dist_apart = Marker::distance(*this->markers[i], *m);
            if(dist_apart >= (avg_width * this->min_marker_distance)
               && dist_apart <= (avg_width * this->max_marker_distance)) {
                return true;
            }
        }
    }
    return false;
}

std::shared_ptr<Marker> Target::getCorner() {
    return this->corner;
}

std::string Target::str() {
    std::stringstream ss;
    cv::Rect rect = this->rect();
    ss << "\"target\": {\"rect\": {\"x\":" << rect.x << ", \"y\":" << rect.y;
    ss << ", \"w\":" << rect.width << ", \"h\":" << rect.height << "}, ";
    ss << "\"angle\": " << this->calc_angle + this->angle_offset << ", \"length\": " << this->calc_length;
    ss << "}";
    return ss.str();
}

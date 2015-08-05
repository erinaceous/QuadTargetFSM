//
// Created by owain on 02/08/15.
//

#include <math.h>
#include <opencv2/opencv.hpp>
#include "TargetFinder.h"

using namespace targetfinder;

/* Target::Target() {
    this->marker_count = 0;
    // this->markers = (Marker*) malloc(sizeof(Marker*) * 3);
    this->markers = new Marker*[3];
    // this->markers = new Marker[3];
} */

/* Target::~Target() {
    free(this->markers);
} */

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
    cv::Point center = this->markers[0]->center();
    int top = center.y, left = center.x;
    int bottom = center.y, right = center.x;
    for(int i=1; i<this->marker_count; i++) {
        cv::Point marker_center = this->markers[i]->center();
        if(marker_center.x < left) {
            left = marker_center.x;
        }
        if(marker_center.x > right) {
            right = marker_center.x;
        }
        if(marker_center.y < top) {
            top = marker_center.y;
        }
        if(marker_center.y > bottom) {
            bottom = marker_center.y;
        }

    }
    return cv::Rect(
            left, top, (right - left), (bottom - top)
    );
}

cv::RotatedRect Target::rotatedRect() {
    cv::Rect rect = this->rect();
    return cv::RotatedRect(
            this->center(), cv::Size2f(rect.width, rect.height), this->calc_angle
    );
}

cv::Point Target::center() {
    cv::Rect rect = this->rect();
    return cv::Point(
            rect.x + (rect.width / 2),
            rect.y + (rect.height / 2)
    );
}

void Target::calcGeometry() {
    float a01 = Marker::angle(*this->markers[0], *this->markers[1]);
    float a02 = Marker::angle(*this->markers[0], *this->markers[2]);
    float a10 = Marker::angle(*this->markers[1], *this->markers[0]);
    float a12 = Marker::angle(*this->markers[1], *this->markers[2]);
    float a20 = Marker::angle(*this->markers[2], *this->markers[0]);
    float a21 = Marker::angle(*this->markers[2], *this->markers[1]);
    float a0 = abs(a01 - a02);
    float a1 = abs(a10 - a12);
    float a2 = abs(a20 - a21);

    /* if(a0 > M_PI_2) a0 = M_PI - a0;
    if(a1 > M_PI_2) a1 = M_PI - a1;
    if(a2 > M_PI_2) a2 = M_PI - a2; */

    float centerx, centery;
    cv::Point center_one, center_two;

    if(a0 >= a1 && a0 >= a2) {
        this->corner = markers[0];
        this->cornerAngle = a0;
        cv::Point center_one = markers[1]->center();
        cv::Point center_two = markers[2]->center();
    }

    if(a1 >= a0 && a1 >= a2) {
        this->corner = markers[1];
        this->cornerAngle = a1;
        cv::Point center_one = markers[0]->center();
        cv::Point center_two = markers[2]->center();
    }

    if(a2 >= a0 && a2 >= a1) {
        this->corner = markers[2];
        this->cornerAngle = a2;
        cv::Point center_one = markers[0]->center();
        cv::Point center_two = markers[1]->center();
    }

    cv::Point new_center = cv::Point(
            (center_one.x + center_two.x) / 2,
            (center_one.y + center_two.y) / 2
    );
    cv::Point corner_center = this->corner->center();
    float dx = corner_center.x - new_center.x;
    float dy = corner_center.y - new_center.y;
    this->calc_angle = atan2(dy, dx);
    this->calc_length = sqrt((dx * dx) + (dy * dy));
}

bool Target::isClose(std::shared_ptr<Marker> m) {
    if(this->marker_count > 2) {
        return false;
    }
    float avg_size, avg_width;
    for(int i=0; i<this->marker_count; i++) {
        avg_size += this->markers[i]->rect().area();
        avg_width += this->markers[i]->xlength();
    }
    avg_size /= (float) this->marker_count;
    avg_width /= (float) this->marker_count;
    float min_size = avg_size * this->marker_size_tolerance;
    float max_size = avg_size / this->marker_size_tolerance;
    float marker_area = m->rect().area();
    if(marker_area >= min_size && marker_area <= max_size) {
        for(int i=0; i<this->marker_count; i++) {
            float dist_apart = Marker::distance(*this->markers[i], *m);
            if(dist_apart >= (avg_width * this->min_marker_distance)
               && dist_apart <= (avg_width * this->max_marker_distance)) {
                return true;
            }
        }
    }
    return false;
}

std::string Target::str() {
    std::stringstream ss;
    cv::Rect rect = this->rect();
    ss << "Target({rect: (" << rect.x << ", " << rect.y << ", " << rect.width << ", " << rect.height << "), ";
    ss << "angle: " << this->calc_angle << ", length: " << this->calc_length;
    ss << ", markers: " << this->marker_count << "})";
    return ss.str();
}
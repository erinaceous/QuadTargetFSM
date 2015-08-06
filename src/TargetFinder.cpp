//
// Created by owain on 02/08/15.
//

#include <iostream>
#include "TargetFinder.h"
#include <opencv2/opencv.hpp>

using namespace targetfinder;

void TargetFinder::setRowStep(int row_step) {
    this->row_step = row_step;
}

void TargetFinder::setMinLength(int min_length) {
    this->min_length = min_length;
}

void TargetFinder::setTolerance(float tolerance) {
    this->tolerance = tolerance;
}

void TargetFinder::setNumBins(int num_bins) {
    this->num_bins = num_bins;
}

void TargetFinder::setMarkerAspectTolerance(float tolerance) {
    this->marker_aspect_tolerance = tolerance;
}

void TargetFinder::setMarkerDistances(float min_distance, float max_distance) {
    this->min_marker_distance = min_distance;
    this->max_marker_distance = max_distance;
}

void TargetFinder::setMarkerSizeTolerance(float tolerance) {
    this->marker_size_tolerance = tolerance;
}

void TargetFinder::setAngleOffset(float angle) {
    this->angle_offset = angle;
}

float TargetFinder::aspect(int width, int height) {
    if(height > width) {
        return ((float) height / (float) width);
    }
    return ((float) width / (float) height);
}

std::vector<PersistentTarget> TargetFinder::getPersistentTargets() {
    return this->finalTargets;
}

std::vector<Target> TargetFinder::doTargetRecognition(cv::Mat input, cv::Mat output, bool show_state) {
    std::vector<std::shared_ptr<Marker>> markers;
    std::vector<cv::Mat> yuv;
    /* cv::split(input, yuv);
    cv::threshold(yuv[0], yuv[0], 255, 255, cv::THRESH_OTSU);
    cv::merge(yuv, input); */

    StateMachine *sm[this->num_bins];
    unsigned int bin_vals[this->num_bins];
    for(int b=0; b<this->num_bins; b++) {
        sm[b] = new StateMachine(input.cols);
        bin_vals[b] = (int) ((255 / this->num_bins) * (b + 0.5));
    }
    StateMachine sm2(input.rows);       // Vertically-scanning state machine

    float aspect = TargetFinder::aspect(input.cols, input.rows);
    int last_center_x = 0;

    for(int y=0; y<input.rows; y += this->row_step) {
        uchar *p = input.ptr(y);
        uchar *o = output.ptr(y);
        for(int b=0; b<this->num_bins; b++) {
            sm[b]->reset(y, (p[0] > bin_vals[b]));
            for (int x = 0; x < input.cols; x++) {
                Marker *m = sm[b]->step(x, y, (p[3 * x] > bin_vals[b]));
                if (show_state && output.rows > 0 && sm[b]->state > 0) {
                    cv::Vec3b color = sm[b]->state_colour();
                    for (int c = 0; c < 3; c++) {
                        o[(3 * x) + c] = color[c];
                    }
                }
                if (m) {
                    bool expanded = false;
                    for (int i = 0; i < markers.size(); i++) {
                        if (markers[i]->contains(*m)) {
                            markers[i]->expand(*m);
                            delete m;
                            expanded = true;
                            break;
                        }
                    }
                    if (!expanded) {
                        cv::Point center = m->center();
                        if (center.x == last_center_x) {
                            continue;
                        }
                        last_center_x = center.x;
                        int w2 = (int) (((m->xlength() / 2.0)) / this->tolerance);
                        int h2 = (int) ((aspect * (m->xlength() / 2.0)) / this->tolerance);
                        int start_y = center.y - h2;
                        int end_y = center.y + h2;
                        if (start_y < 0) {
                            start_y = 0;
                        }
                        if (start_y >= input.rows) {
                            start_y = input.rows - 1;
                        }
                        if (end_y < 0) {
                            end_y = 0;
                        }
                        if (end_y >= input.rows) {
                            end_y = input.rows - 1;
                        }
                        Marker *m2;
                        p = input.ptr(start_y);
                        sm2.reset(center.x, (p[center.x * 3] > bin_vals[b]));
                        for (int y2 = start_y; y2 < end_y; y2++) {
                            p = input.ptr(y2);
                            m2 = sm2.step(y2, center.x, (p[center.x * 3] > bin_vals[b]));
                            if (show_state && output.rows > 0 && sm2.state > 0) {
                                cv::Vec3b color = sm2.state_colour();
                                o = output.ptr(y2);
                                for (int c = 0; c < 3; c++) {
                                    o[(center.x * 3) + c] = color[c];
                                }
                            }
                            if (m2) {
                                m->expand(*m2, true);
                                break;
                            }
                        }
                        if (m2) {
                            int min_pixels = this->min_length * StateMachine::NUM_STATES;
                            float marker_aspect = TargetFinder::aspect(m->xlength(), m->ylength());
                            float min_marker_aspect = this->marker_aspect_tolerance;
                            float max_marker_aspect = 1.0 + this->marker_aspect_tolerance;
                            if (m->ylength() >= min_pixels && m->xlength() >= min_pixels
                                && marker_aspect >= min_marker_aspect && marker_aspect <= max_marker_aspect) {
                                markers.push_back(std::shared_ptr<Marker>(m));
                            }
                            delete m2;
                        }
                        o = output.ptr(y);
                        p = input.ptr(y);
                    }
                }
            }
        }
    }

    std::vector<Target> targets;

    for(int i=0; i<markers.size(); i++) {
        cv::rectangle(output, markers[i]->rect(), cv::Scalar(0, 0, 0), 1);
        bool found_target = false;
        for(int t=0; t<targets.size(); t++) {
            if(targets[t].isClose(markers[i])) {
                targets[t].addMarker(markers[i]);
                found_target = true;
                break;
            }
        }
        if(!found_target) {
            Target target;
            target.min_marker_distance = this->min_marker_distance;
            target.max_marker_distance = this->max_marker_distance;
            target.marker_size_tolerance = this->marker_size_tolerance;
            target.angle_offset = this->angle_offset;
            target.addMarker(markers[i]);
            targets.push_back(target);
        }
    }
    for(int t=0; t<targets.size(); t++) {
        targets[t].calc_valid = targets[t].valid();
        if(targets[t].calc_valid) {
            targets[t].calcGeometry();
        }
    }

    return targets;
}

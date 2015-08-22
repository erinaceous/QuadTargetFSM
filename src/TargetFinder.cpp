//
// Created by owain on 02/08/15.
//

#include <math.h>
#include <iostream>
#include "include/TargetFinder.hpp"
#include "include/Marker.hpp"

using namespace targetfinder;

void TargetFinder::setRowStep(int row_step) {
    this->row_step = row_step;
}

void TargetFinder::setMinLength(int min_length) {
    this->min_length = min_length;
}

void TargetFinder::setTolerance(double tolerance) {
    this->tolerance = tolerance;
}

void TargetFinder::setNumBins(int num_bins) {
    this->num_bins = num_bins;
}

void TargetFinder::setMarkerAspectTolerance(double tolerance) {
    this->marker_aspect_tolerance = tolerance;
}

void TargetFinder::setMarkerDistances(double min_distance, double max_distance) {
    this->min_marker_distance = min_distance;
    this->max_marker_distance = max_distance;
}

void TargetFinder::setMarkerSizeTolerance(double tolerance) {
    this->marker_size_tolerance = tolerance;
}

void TargetFinder::setAngleOffset(double angle) {
    this->angle_offset = angle;
}

void TargetFinder::setVarianceThreshold(double variance) {
    this->variance_threshold = variance;
}

void TargetFinder::setAlpha(double alpha) {
    this->alpha = alpha;
}

void TargetFinder::shouldFilterTexture(bool filter) {
    this->filter_texture = filter;
}

double TargetFinder::aspect(int width, int height) {
    if(height > width) {
        return ((double) height / (double) width);
    }
    return ((double) width / (double) height);
}

std::vector<PersistentTarget> TargetFinder::getPersistentTargets() {
    return this->finalTargets;
}

std::vector<Target> TargetFinder::doTargetRecognition(cv::Mat input, cv::Mat output,
                                                      bool show_state,
                                                      bool show_markers,
                                                      std::string *marker_info) {
    std::vector<std::shared_ptr<Marker>> markers;

    StateMachine *sm[this->num_bins];
    unsigned int bin_vals[this->num_bins];
    for(int b=0; b<this->num_bins; b++) {
        sm[b] = new StateMachine(
                input.cols,
                this->tolerance,
                this->min_length
        );
        bin_vals[b] = (int) ((255 / this->num_bins) * (b + 0.5));
    }
    StateMachine sm2(
            input.rows,
            this->tolerance,
            this->min_length
    );       // Vertically-scanning state machine

    double aspect = TargetFinder::aspect(input.cols, input.rows);
    double alpha1 = (1.0 / (((double) input.cols) / 8.0));
    double alpha2 = 8.0;
    double local_value = 0.0, delta = 0.0;
    double global_delta = 0.0, local_delta = 0.0;
    double last_value = 0;
    int last_center_x = 0;
    Marker *m = nullptr;
    bool lacking_texture = true;

    for(int y=0; y<input.rows; y += this->row_step) {
        uchar *p = input.ptr(y);
        uchar *o = output.ptr(y);

        global_delta = 0.0;
        local_delta = 0.0;

        for(int x=0; x<input.cols; x++) {
            if(this->filter_texture) {
                local_value = (p[(3 * x)] + p[(3 * x) + 1] + p[(3 * x) + 2]) / 3.0;
                delta = fabs(local_value - last_value);
                global_delta =
                        (alpha1 * delta) + ((1.0 - alpha1) * global_delta);
                local_delta = (local_delta / alpha2) + delta;
                last_value = local_value;
                lacking_texture = local_delta <= MIN(1.0, global_delta);
            }

            for(int b=0; b<this->num_bins; b++) {
                unsigned int thresh_val = bin_vals[b];
                bool sm_value = (p[(3 * x)] >= thresh_val);

                if(x == 0) {
                    sm[b]->reset(y, sm_value, lacking_texture);
                } else {
                    m = sm[b]->step(x, y, sm_value, lacking_texture);
                }

                if (show_state && output.rows > 0 && sm[b]->state > 0) {
                    cv::Vec3b color = sm[b]->state_colour();
                    for (int c=0; c<3; c++) {
                        o[(3 * x) + c] = color[c];
                    }
                }

                /* for(int c=0; c<3; c++) {
                    o[(3 * x) + c] = p[(3 * x) + 2];
                } */

                if (m) {
                    bool expanded = false;
                    for (int i=0; i<markers.size(); i++) {
                        if (markers[i]->contains(*m)) {
                            markers[i]->expand(*m);
                            delete m;
                            expanded = true;
                            break;
                        }
                    }
                    if (!expanded) {
                        cv::Rect rect = m->rect();
                        cv::Point center = m->center();
                        if (center.x == last_center_x) {
                            continue;
                        }
                        last_center_x = center.x;
                        int h2 = (int) ((aspect * (m->xlength() * 2.0)));
                        int start_x = center.x - this->row_step;
                        int end_x = center.x + this->row_step;
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
                        if(start_x < 0) {
                            start_x = 0;
                        }
                        if(start_x >= input.cols) {
                            start_x = input.cols - 1;
                        }
                        if(end_x < 0) {
                            end_x = 0;
                        }
                        if(end_x >= input.cols) {
                            end_x = input.cols - 1;
                        }
                        Marker *m2;
                        p = input.ptr(start_y);
                        for(int x2 = start_x; x2 <= end_x; x2 += this->row_step) {
                            sm2.reset(x2, (p[(x2 * 3)] > thresh_val));
                            for (int y2 = start_y; y2 < end_y; y2++) {
                                p = input.ptr(y2);
                                m2 = sm2.step(y2, x2,
                                              (p[(x2 * 3)] > thresh_val));
                                if (show_state && output.rows > 0 &&
                                                  sm2.state > 0) {
                                    cv::Vec3b color = sm2.state_colour();
                                    o = output.ptr(y2);
                                    for (int c = 0; c < 3; c++) {
                                        o[(x2 * 3) + c] = color[c];
                                    }
                                }
                                if (m2) {
                                    m->expand(*m2, true);
                                    break;
                                }
                            }
                            if(m2) {
                                break;
                            }
                        }
                        if (m2) {
                            int min_pixels = this->min_length * StateMachine::NUM_STATES;
                            double marker_aspect = TargetFinder::aspect(m->xlength(), m->ylength());
                            double min_marker_aspect = this->marker_aspect_tolerance;
                            double max_marker_aspect = 1.0 + this->marker_aspect_tolerance;
                            if (m->ylength() >= min_pixels
                                && m->xlength() >= min_pixels
                                && marker_aspect >= min_marker_aspect
                                && marker_aspect <= max_marker_aspect) {
                                markers.push_back(
                                        std::shared_ptr<Marker>(m));
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
        if(&output && show_markers) {
            cv::rectangle(output, markers[i]->rect(), cv::Scalar(0, 0, 0), 1);
        }
        if(marker_info != nullptr && markers[i] != nullptr) {
            long rpos = marker_info->rfind('/') + 1;
            std::string marker_base = marker_info->substr(rpos, marker_info->length());
            std::cout << marker_base << ", " << markers[i]->str() << std::endl;
        }
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

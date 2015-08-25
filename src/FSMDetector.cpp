//
// Created by owain on 8/25/15.
//

#include <memory>
#include <opencv2/opencv.hpp>
#include <Utils.h>
#include "include/StateMachine.hpp"
#include "include/MarkerDetector.hpp"
#include "include/Marker.hpp"

namespace targetfinder {
    class FSMDetector : public MarkerDetector {
        public:
            static constexpr double MARKER_ASPECT_TOLERANCE = 0.9;
            static constexpr int ROW_STEP = 1;
            static constexpr double MIN_THRESHOLD = 25.0;
            static constexpr double ALPHA = 0.00001;

            void setMinThreshold(int thresh) {
                this->min_threshold = thresh;
            }

            void setRowStep(int step) {
                this->row_step = step;
            }

            void setMinLength(int length) {
                this->min_length = length;
            }

            void setSmoothness(double smoothness) {
                this->alpha = smoothness;
            }

            void setTolerance(double tolerance) {
                this->tolerance = tolerance;
            }

            void setMarkerAspectTolerance(double tolerance) {
                this->marker_aspect_tolerance = tolerance;
            }

            std::vector<std::shared_ptr<Marker>> detect(
                    cv::Mat input,
                    cv::Mat output,
                    bool show_state
            ) {
                markers.clear();

                StateMachine sm(
                        input.cols,
                        this->tolerance,
                        this->min_length
                );
                StateMachine sm2(
                        input.rows,
                        this->tolerance,
                        this->min_length
                );       // Vertically-scanning state machine

                double aspect = _aspect(input.cols, input.rows);
                double alpha1 = ((1.0 / ((double) input.cols / this->alpha)));
                double global_delta = 0.0;
                int last_center_x = 0;
                Marker *m = nullptr;
                double threshold = 0.0;
                unsigned char horiz_last = 0;
                unsigned char horiz = 0;
                int horiz_diff = 0;
                bool sm_value = false;

                for(int y=0; y<input.rows; y += this->row_step) {
                    uchar *p = input.ptr(y);
                    uchar *o = output.ptr(y);

                    sm.reset(y);

                    for(int x=0; x<input.cols; x++) {
                        horiz = p[(3 * x)];
                        horiz_diff = horiz - horiz_last;
                        if((horiz_diff > threshold) || (horiz_diff < -threshold)) {
                            if(horiz_diff < 0) {
                                sm_value = false;
                            } else {
                                sm_value = true;
                            }
                        }
                        global_delta = (alpha1 * horiz_diff) + ((1.0 - alpha1) * global_delta);
                        threshold = (MAX(this->min_threshold, global_delta));
                        horiz_last = horiz;

                        m = sm.step(x, y, sm_value);

                        if (show_state && output.rows > 0 && sm.state > 0) {
                            cv::Vec3b color = sm.state_colour();
                            for (int c=0; c<3; c++) {
                                o[(3 * x) + c] = color[c];
                            }
                        }

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
                                int h2 = (int) ((aspect * (m->xlength())));
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
                                    unsigned char vert = 0;
                                    unsigned char vert_last = 0;
                                    int vert_diff = 0;

                                    bool sm2_value = false;

                                    sm2.reset(x2);
                                    for (int y2 = start_y; y2 < end_y; y2++) {
                                        p = input.ptr(y2);

                                        vert = p[(3 * x2)];
                                        vert_diff = vert - vert_last;
                                        vert_last = vert;

                                        if((vert_diff > threshold) || (vert_diff < -threshold)) {
                                            if(vert_diff < 0) {
                                                sm2_value = false;
                                            } else {
                                                sm2_value = true;
                                            }
                                        }

                                        m2 = sm2.step(y2, x2, sm2_value);
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
                                    int min_pixels = (int) ((this->min_length * StateMachine::NUM_STATES) * this->tolerance);
                                    double marker_aspect = _aspect(m->xlength(), m->ylength());
                                    double min_marker_aspect = 1.0 - this->marker_aspect_tolerance;
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

                return markers;
            }

        private:
            int row_step = ROW_STEP;
            int min_threshold = MIN_THRESHOLD;
            int min_length = StateMachine::MIN_LENGTH;
            double marker_aspect_tolerance = MARKER_ASPECT_TOLERANCE;
            double alpha = ALPHA;
            double tolerance = StateMachine::TOLERANCE;
            std::vector<std::shared_ptr<Marker>> markers;
    };
}

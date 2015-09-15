//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_MARKER_HPP
#define QUADTARGETFSM_MARKER_HPP

#include <memory>
#include <opencv2/opencv.hpp>

namespace targetfinder {
    class Marker {
        friend class Target;
        friend class TargetFinder;
        friend class StateMachine;
    public:
        Marker(int start_x, int end_x, int start_y, int end_y, int black_count, int white_count, int center_count);
        void expand(Marker other, bool rotate=false);
        void expand(int start_x, int end_x, int start_y, int end_y,
                    int black_count=-1, int white_count=-1, int center_count=-1);
        cv::Point center();
        cv::Point2f centerf();
        int xlength();
        int ylength();
        bool contains(Marker other);
        bool contains(cv::Point other);
        bool contains(double x, double y);
        double distance(Marker other);
        static double distance(Marker one, Marker two);
        static double angle(Marker one, Marker two);
        cv::Rect rect();
        std::shared_ptr<Marker> cloned_shared_ptr();
        std::string str();
    protected:
        int start_x, end_x, start_y, end_y, black_count, white_count, center_count;
        int calc_xlength, calc_ylength;
        cv::Point center_point;
        bool updated = true, calc_valid = true;
        int expanded_count = 0;
    };

}

#endif //QUADTARGETFSM_MARKER_HPP

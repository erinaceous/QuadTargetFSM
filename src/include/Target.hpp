//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_TARGET_HPP
#define QUADTARGETFSM_TARGET_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include "Marker.hpp"

namespace targetfinder {

    class Target {
        friend class Marker;
        friend class TargetFinder;
    public:
        static constexpr float MIN_MARKER_DISTANCE = 0.5;
        static constexpr float MAX_MARKER_DISTANCE = 5.0;
        static constexpr float MARKER_SIZE_TOLERANCE = 0.7;

        // Target();
        // ~Target();
        bool valid();
        bool addMarker(std::shared_ptr<Marker> m);
        float angle();
        float length();
        cv::Rect rect();
        cv::RotatedRect rotatedRect();
        cv::Point center();
        bool isClose(std::shared_ptr<Marker> m);
        std::string str();
        std::shared_ptr<Marker> getCorner();
        std::shared_ptr<Marker>* getMarkers();

        bool calc_valid;
    protected:
        std::shared_ptr<Marker> markers[3];
        int marker_count = 0;
        float calc_angle, calc_length;
        cv::Rect bounding_box;
        cv::Point center_point;
        bool updated = true;
        void calcGeometry();
        std::shared_ptr<Marker> corner;
        float corner_angle, angle_offset;
        float min_marker_distance, max_marker_distance, marker_size_tolerance;
    };

}

#endif //QUADTARGETFSM_TARGET_HPP

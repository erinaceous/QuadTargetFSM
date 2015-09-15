//
// Created by owain on 8/25/15.
//

#ifndef QUADTARGETFSM_MARKERDETECTOR_HPP
#define QUADTARGETFSM_MARKERDETECTOR_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include "Marker.hpp"

namespace targetfinder {
/**
 * Abstract class for describing a marker detector. This is so that the rest of
 * the target finding code can be agnostic to whichever method is in use --
 * finite state machine pattern detection, or the Cascade Classification
 * algorithms.
 *
 * FSMDetector and CascadeDetector inherit from this class.
 *
 */

    class MarkerDetector {
        friend class Marker;

        public:
            virtual std::vector <std::shared_ptr<Marker>> detect(
                    cv::Mat input,
                    cv::Mat output,
                    bool show_state = false
            ) {};
            virtual std::string getName() { return "null"; }
    };

}

#endif //QUADTARGETFSM_MARKERDETECTOR_HPP

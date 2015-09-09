//
// Created by owain on 02/08/15.
//

#include <math.h>
#include <iostream>
#include "include/TargetFinder.hpp"
#include "include/Marker.hpp"

using namespace targetfinder;

void TargetFinder::setMarkerDistances(float min_distance, float max_distance) {
    this->min_marker_distance = min_distance;
    this->max_marker_distance = max_distance;
}

void TargetFinder::setMarkerSizeTolerance(float tolerance) {
    this->marker_size_tolerance = tolerance;
}

std::vector<Target> TargetFinder::groupTargets(cv::Mat input, cv::Mat output,
                                                      std::vector<std::shared_ptr<Marker>> markers,
                                                      bool show_markers,
                                                      std::string *marker_info) {
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

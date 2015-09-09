//
// Created by owain on 02/08/15.
//

#ifndef QUADTARGETFSM_TARGETFINDER_H
#define QUADTARGETFSM_TARGETFINDER_H

#include <memory>
#include <opencv2/core/core.hpp>
#include "StateMachine.hpp"
#include "Target.hpp"
#include "PersistentTarget.hpp"

namespace targetfinder {

    class TargetFinder {
        friend class Marker;
        friend class Target;
        public:
            std::vector<Target> groupTargets(cv::Mat input, cv::Mat output,
                                             std::vector<std::shared_ptr<Marker>> markers,
                                             bool show_markers=false,
                                             std::string *marker_info=nullptr);
            void setMarkerDistances(float min_distance, float max_distance);
            void setMarkerSizeTolerance(float tolerance);

        protected:
            float min_marker_distance = Target::MIN_MARKER_DISTANCE;
            float max_marker_distance = Target::MAX_MARKER_DISTANCE;
            float marker_size_tolerance = Target::MARKER_SIZE_TOLERANCE;
    };

}


#endif //QUADTARGETFSM_TARGETFINDER_H

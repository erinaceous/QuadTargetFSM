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
            static constexpr int THRESH_BINS = 1;
            static constexpr double MARKER_ASPECT_TOLERANCE = 0.9;
            static constexpr int ROW_STEP = 1;
            static constexpr double VARIANCE_THRESHOLD = 0.33;

            std::vector<Target> doTargetRecognition(cv::Mat input, cv::Mat output,
                                                    bool show_state=false,
                                                    bool show_markers=false,
                                                    std::string *marker_info=nullptr);
            std::vector<PersistentTarget> getPersistentTargets();
            void setRowStep(int row_step);
            void setMinLength(int min_length);
            void setTolerance(double tolerance);
            void setNumBins(int num_bins);
            void setMarkerAspectTolerance(double tolerance);
            void setMarkerDistances(double min_distance, double max_distance);
            void setMarkerSizeTolerance(double tolerance);
            void setAngleOffset(double angle);
            void setVarianceThreshold(double variance);
            void setAlpha(double alpha);
            void shouldFilterTexture(bool filter);
            Marker* getCorner();

        protected:
            std::vector<PersistentTarget> finalTargets;
            int row_step = ROW_STEP;
            int min_length = StateMachine::MIN_LENGTH;
            int num_bins = THRESH_BINS;
            double tolerance = StateMachine::TOLERANCE;
            double marker_aspect_tolerance = MARKER_ASPECT_TOLERANCE;
            double min_marker_distance = Target::MIN_MARKER_DISTANCE;
            double max_marker_distance = Target::MAX_MARKER_DISTANCE;
            double marker_size_tolerance = Target::MARKER_SIZE_TOLERANCE;
            double angle_offset = 0.0;
            double alpha = 0.33;
            double variance_threshold = VARIANCE_THRESHOLD;
            bool filter_texture = false;

        private:
            static double aspect(int width, int height);
    };

}


#endif //QUADTARGETFSM_TARGETFINDER_H

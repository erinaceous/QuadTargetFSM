//
// Created by owain on 02/08/15.
//

#ifndef QUADTARGETFSM_TARGETFINDER_H
#define QUADTARGETFSM_TARGETFINDER_H

#include <memory>
#include <opencv2/core/core.hpp>

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
            bool contains(float x, float y);
            float distance(Marker other);
            static float distance(Marker one, Marker two);
            static float angle(Marker one, Marker two);
            cv::Rect rect();
            std::shared_ptr<Marker> cloned_shared_ptr();
        protected:
            int start_x, end_x, start_y, end_y, black_count, white_count, center_count;
            int calc_xlength, calc_ylength;
            cv::Point center_point;
            bool updated = true, calc_valid = true;
    };

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
            float cornerAngle;
            float min_marker_distance, max_marker_distance, marker_size_tolerance;
    };

    class PersistentTarget {
        friend class Target;
        friend class TargetFinder;
        public:
            static constexpr int LIFETIME = 5;
            void update(cv::RotatedRect new_rotated_rect, float influence=0.5);
            static float similarity(cv::RotatedRect one, cv::RotatedRect two);
            float similarity(cv::RotatedRect other);
            cv::RotatedRect getRotatedRect();
            std::string str();
        protected:
            cv::RotatedRect rotatedRect;
            int lifetime = LIFETIME;
    };

    class StateMachine {
        friend class Marker;
        public:
            static constexpr int MIN_LENGTH = 1;
            static constexpr float TOLERANCE = 0.5;
            static constexpr int NUM_STATES = 7;

            StateMachine(int input_length=0, float tolerance=TOLERANCE, int min_pixels=MIN_LENGTH, float aspect=1.0);
            Marker* step(int x, int y, bool value);
            void reset(int y, bool value=false);
            cv::Vec3b state_colour();
            void setInputLength(int input_length);
            int state;
        protected:
            bool in_bounds(int count, float scale=1.0);
            bool valid_transition(int x, bool value);
            void transition(int x, int y, bool value);

            bool last_value;
            int first_x, last_x, y, input_length, min_pixels, max_pixels;
            int pixel_counts[NUM_STATES];
            float tolerance, aspect, min_bound, max_bound;
    };

    class TargetFinder {
        friend class Marker;
        friend class Target;
        public:
            static constexpr int THRESH_BINS = 1;
            static constexpr float MARKER_ASPECT_TOLERANCE = 0.9;

            std::vector<Target> doTargetRecognition(cv::Mat input, cv::Mat output, bool show_state=false);
            std::vector<PersistentTarget> getPersistentTargets();
            void setMinLength(int min_length);
            void setTolerance(float tolerance);
            void setNumBins(int num_bins);
            void setMarkerAspectTolerance(float tolerance);
            void setMarkerDistances(float min_distance, float max_distance);
            void setMarkerSizeTolerance(float tolerance);

        protected:
            std::vector<PersistentTarget> finalTargets;
            int min_length = StateMachine::MIN_LENGTH;
            int num_bins = THRESH_BINS;
            float tolerance = StateMachine::TOLERANCE;
            float marker_aspect_tolerance = MARKER_ASPECT_TOLERANCE;
            float min_marker_distance = Target::MIN_MARKER_DISTANCE;
            float max_marker_distance = Target::MAX_MARKER_DISTANCE;
            float marker_size_tolerance = Target::MARKER_SIZE_TOLERANCE;

        private:
            static float aspect(int width, int height);
    };

}


#endif //QUADTARGETFSM_TARGETFINDER_H

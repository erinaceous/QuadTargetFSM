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
            bool contains(double x, double y);
            double distance(Marker other);
            static double distance(Marker one, Marker two);
            static double angle(Marker one, Marker two);
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
            static constexpr double MIN_MARKER_DISTANCE = 0.5;
            static constexpr double MAX_MARKER_DISTANCE = 5.0;
            static constexpr double MARKER_SIZE_TOLERANCE = 0.7;

            // Target();
            // ~Target();
            bool valid();
            bool addMarker(std::shared_ptr<Marker> m);
            double angle();
            double length();
            cv::Rect rect();
            cv::RotatedRect rotatedRect();
            cv::Point center();
            bool isClose(std::shared_ptr<Marker> m);
            std::string str();

            bool calc_valid;
        protected:
            std::shared_ptr<Marker> markers[3];
            int marker_count = 0;
            double calc_angle, calc_length;
            cv::Rect bounding_box;
            cv::Point center_point;
            bool updated = true;
            void calcGeometry();
            std::shared_ptr<Marker> corner;
            double corner_angle, angle_offset;
            double min_marker_distance, max_marker_distance, marker_size_tolerance;
    };

    class PersistentTarget {
        friend class Target;
        friend class TargetFinder;
        public:
            void update(Target *new_target, int64 timestamp, double influence);
            bool alive(int64 timestamp);
            double similarity(Target other);
            double angle();
            cv::Rect rect();
            cv::Point center();
            void setTimeout(int64 ticks);
            void setAngleOffset(double angle);
            std::string str();
            int age();
        protected:
            int64 timeout_ticks;
            int64 last_updated;
            int lifetime;
            int x, y, width, height;
            double calc_angle, angle_offset;
    };

    class StateMachine {
        friend class Marker;
        public:
            static constexpr int MIN_LENGTH = 1;
            static constexpr double TOLERANCE = 0.5;
            static constexpr int NUM_STATES = 7;

            StateMachine(int input_length=0, double tolerance=TOLERANCE, int min_pixels=MIN_LENGTH);
            Marker* step(int x, int y, bool value);
            void reset(int y, bool value=false);
            cv::Vec3b state_colour();
            void setInputLength(int input_length);
            int state;

        protected:
            bool in_bounds(int count, int scale=1);
            bool valid_transition(int x, bool value);
            void transition(int x, int y, bool value);

            bool last_value;
            int first_x, last_x, y, input_length, min_pixels, max_pixels;
            int min_bound, max_bound;
            int pixel_counts[NUM_STATES];
            double tolerance;


    };

    class CameraModel {
        friend class TargetFinder;
        public:
            static constexpr double FOCAL_LENGTH = 3.6;
            static constexpr double SENSOR_WIDTH = 3.67;
            static constexpr double SENSOR_HEIGHT = 2.74;
            static constexpr double MAGNIFICATION_FACTOR = 1.0;
            static constexpr double TARGET_WIDTH = 594;
            static constexpr double TARGET_HEIGHT = 420;

            double distance(int calc_width, int calc_height);
            CameraModel(int image_width, int image_height, double target_width=TARGET_WIDTH,
                        double target_height=TARGET_HEIGHT, double focal_length=FOCAL_LENGTH,
                        double magnification_factor=MAGNIFICATION_FACTOR, double sensor_width=SENSOR_WIDTH,
                        double sensor_height=SENSOR_HEIGHT);
        protected:
            int image_width, image_height;
            double target_width = TARGET_WIDTH, target_height = TARGET_HEIGHT;
            double focal_length = FOCAL_LENGTH, magnification_factor = MAGNIFICATION_FACTOR;
            double sensor_width = SENSOR_WIDTH, sensor_height = SENSOR_HEIGHT;
    };

    class TargetFinder {
        friend class Marker;
        friend class Target;
        public:
            static constexpr int THRESH_BINS = 1;
            static constexpr double MARKER_ASPECT_TOLERANCE = 0.9;
            static constexpr int ROW_STEP = 1;

            std::vector<Target> doTargetRecognition(cv::Mat input, cv::Mat output, bool show_state=false);
            std::vector<PersistentTarget> getPersistentTargets();
            void setRowStep(int row_step);
            void setMinLength(int min_length);
            void setTolerance(double tolerance);
            void setNumBins(int num_bins);
            void setMarkerAspectTolerance(double tolerance);
            void setMarkerDistances(double min_distance, double max_distance);
            void setMarkerSizeTolerance(double tolerance);
            void setAngleOffset(double angle);

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

        private:
            static double aspect(int width, int height);
    };

    class Navigator {
        public:
            static constexpr double ROTATION_DEADZONE = 30.0;
            static constexpr double AXES_DEADZONE = 0.1;
            static constexpr double UPDATE_RATE = 0.1;
            Navigator(int width, int height, double rotation_deadzone=ROTATION_DEADZONE,
                      double horizontal_deadzone=AXES_DEADZONE, double vertical_deadzone=AXES_DEADZONE,
                      double update_rate=UPDATE_RATE);
            bool update(cv::Point target, double angle, double distance, int age, double deltatime);
            void update(double deltatime);
            double horizontal();
            double vertical();
            double alt();
            double rotation();
            cv::Point image_point(int axis, int length=150);
            std::string str();
            void setPIDs(
                    double pitch_P, double pitch_I, double pitch_D,
                    double roll_P, double roll_I, double roll_D,
                    double yaw_P=0.1, double yaw_I=0.1, double yaw_D=0.0,
                    double throt_P=0.1, double throt_I=0.1, double throt_D=0.0
            );

            class PID {
                public:
                    PID(double Kp=0.9, double Ki=0.1, double Kd=0.0);
                    double step(double setpoint, double value, double deltatime);
                    double Kp, Ki, Kd;
                    void reset(double value);
                protected:
                    double previous_error;
                    double integral;
            };
        protected:
            int width, height;
            cv::Point image_center;
            double rotation_deadzone, horizontal_deadzone, vertical_deadzone, angle, distance, alpha, x, y;
            PID *pitch_pid, *roll_pid, *throttle_pid, *yaw_pid;
    };

}


#endif //QUADTARGETFSM_TARGETFINDER_H

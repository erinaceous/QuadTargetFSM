//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_NAVIGATOR_HPP
#define QUADTARGETFSM_NAVIGATOR_HPP

namespace targetfinder {

    class Navigator {
    public:
        static constexpr double ROTATION_DEADZONE = 30.0;
        static constexpr double AXES_DEADZONE = 0.1;
        static constexpr double UPDATE_RATE = 1.0;
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
        double rotation_deadzone, horizontal_deadzone, vertical_deadzone,
                angle, distance, alpha, x, y, throttle;
        PID *pitch_pid, *roll_pid, *throttle_pid, *yaw_pid;
        bool should_flip = false;
    };

}

#endif //QUADTARGETFSM_NAVIGATOR_HPP

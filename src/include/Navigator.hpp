//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_NAVIGATOR_HPP
#define QUADTARGETFSM_NAVIGATOR_HPP

namespace targetfinder {

    class Navigator {
    public:
        static constexpr float ROTATION_DEADZONE = 30.0;
        static constexpr float AXES_DEADZONE = 0.1;
        static constexpr float UPDATE_RATE = 1.0;
        Navigator(int width, int height, float rotation_deadzone=ROTATION_DEADZONE,
                  float horizontal_deadzone=AXES_DEADZONE, float vertical_deadzone=AXES_DEADZONE,
                  float update_rate=UPDATE_RATE);
        bool update(cv::Point target, float angle, float distance, int age, float deltatime);
        void update(float deltatime);
        float horizontal();
        float vertical();
        float alt();
        float rotation();
        cv::Point image_point(int axis, int length=150);
        std::string str();
        void setPIDs(
                float pitch_P, float pitch_I, float pitch_D,
                float roll_P, float roll_I, float roll_D,
                float yaw_P=0.1, float yaw_I=0.1, float yaw_D=0.0,
                float throt_P=0.1, float throt_I=0.1, float throt_D=0.0
        );

        class PID {
        public:
            PID(float Kp=0.9, float Ki=0.1, float Kd=0.0);
            float step(float setpoint, float value, float deltatime);
            float Kp, Ki, Kd;
            void reset(float value);
        protected:
            float previous_error;
            float integral;
        };
    protected:
        int width, height;
        cv::Point image_center;
        float rotation_deadzone, horizontal_deadzone, vertical_deadzone,
                angle, distance, alpha, x, y, throttle;
        PID *pitch_pid, *roll_pid, *throttle_pid, *yaw_pid;
        bool should_flip = false;
    };

}

#endif //QUADTARGETFSM_NAVIGATOR_HPP

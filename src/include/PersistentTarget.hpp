//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_PERSISTENTTARGET_HPP
#define QUADTARGETFSM_PERSISTENTTARGET_HPP

#include <opencv2/opencv.hpp>

namespace targetfinder {

    class PersistentTarget {
        friend class Target;
        friend class TargetFinder;
    public:
        void update(Target *new_target, int64 timestamp, double influence);
        bool alive(int64 timestamp);
        double similarity(Target *other);
        double angle();
        cv::Rect rect();
        cv::Point center();
        void setTimeout(int64 ticks);
        void setAngleOffset(double angle);
        std::string str();
        int age();
        double distance(Target *other);
        double velocity();
        void tick();
    protected:
        int64 timeout_ticks;
        int64 last_updated;
        int lifetime;
        int x, y, width, height;
        double calc_angle, angle_offset;
        bool calc_alive = true;
        double calc_velocity = 0.0;
    };

}

#endif //QUADTARGETFSM_PERSISTENTTARGET_HPP

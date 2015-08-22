//
// Created by owain on 8/22/15.
//

#ifndef QUADTARGETFSM_STATEMACHINE_HPP
#define QUADTARGETFSM_STATEMACHINE_HPP

#include "../include/Marker.hpp"

namespace targetfinder {

    class StateMachine {
        friend class Marker;
    public:
        static constexpr int MIN_LENGTH = 1;
        static constexpr double TOLERANCE = 0.5;
        static constexpr int NUM_STATES = 7;

        StateMachine(int input_length=0, double tolerance=TOLERANCE, int min_pixels=MIN_LENGTH);
        Marker* step(int x, int y, bool value1, bool value2=true);
        void reset(int y, bool value1=false, bool value2=true);
        cv::Vec3b state_colour();
        void setInputLength(int input_length);
        int state;

    protected:
        bool in_bounds(int count, int scale=1);
        bool valid_transition(int x, bool value1, bool value2=true);
        void transition(int x, int y, bool value1, bool value2=true);

        bool last_value1, last_value2;
        int first_x, last_x, y, input_length, min_pixels, max_pixels;
        int min_bound, max_bound;
        int pixel_counts[NUM_STATES];
        double tolerance;


    };

}

#endif //QUADTARGETFSM_STATEMACHINE_HPP

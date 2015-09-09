//
// Created by owain on 06/08/15.
//

#ifndef QUADTARGETFSM_UTILS_H
#define QUADTARGETFSM_UTILS_H

#include <math.h>

#define _debug(x) std::cerr << x << std::endl

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

static float _angle(float ax, float ay, float bx, float by) {
    if(bx - ax == 0 || by - ay == 0) {
        return 0.0;
    }
    return atan2(bx - ax, by - ay);
}

static float _degrees(float radians) {
    return (radians * 180.0) / M_PI;
}

static float _aspect(float a1, float a2) {
    float a = MAX(a1, a2);
    float b = MIN(a1, a2);
    return b / a;
}

static float _distance(float ax, float bx, float ay, float by) {
    return sqrt(
            pow(bx - ax, 2) + pow(by - ay, 2)
    );
}

#endif

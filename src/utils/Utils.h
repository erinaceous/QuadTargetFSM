//
// Created by owain on 06/08/15.
//

#ifndef QUADTARGETFSM_UTILS_H
#define QUADTARGETFSM_UTILS_H

#include <math.h>

#define _debug(x) std::cerr << x << std::endl

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

static double _angle(double ax, double ay, double bx, double by) {
    if(bx - ax == 0 || by - ay == 0) {
        return 0.0;
    }
    return atan2(bx - ax, by - ay);
}

static double _degrees(double radians) {
    return (radians * 180.0) / M_PI;
}

static double _aspect(double a1, double a2) {
    double a = MAX(a1, a2);
    double b = MIN(a1, a2);
    return b / a;
}

static double _distance(double ax, double bx, double ay, double by) {
    return sqrt(
            pow(bx - ax, 2) + pow(by - ay, 2)
    );
}

#endif

//
// Created by owain on 06/08/15.
//

#include <math.h>

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

static double _angle(double ax, double ay, double bx, double by) {
    return atan2(bx - ax, by - ay);
}

static double _degrees(double radians) {
    return remainder(radians * (180.0 / M_PI), 360.0);
}
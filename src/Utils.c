//
// Created by owain on 06/08/15.
//

#include <math.h>

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

static float _angle(float ax, float ay, float bx, float by) {
    return atan2(bx - ax, by - ay);
}

static float _degrees(float radians) {
    return remainder(radians * (180.0 / M_PI), 360.0);
}
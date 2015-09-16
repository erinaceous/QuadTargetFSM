//
// Created by owain on 9/11/15.
//

#include "PiCamera.hpp"

#ifndef PI_BUILD
// If we're not compiling on Pi, leave these as stub methods.

PiCamera::PiCamera(int idx) {
    std::cerr << "Pi Camera interface not implemented! Use the normal V4L2";
    std::cerr << " webcam input for now." << std::endl;
    return;
}

bool PiCamera::isOpened() {
    return false;
}

bool PiCamera::read(cv::Mat &image) {
    return false;
}

bool PiCamera::set(int propId, double value) {
    return false;
}

#else

PiCamera::PiCamera(int idx) {
    std::cerr << "Pi Camera interface STILL not implemented! Use the normal";
    std::cerr << "V4L2 webcam input for now." << std::endl;
}

bool PiCamera::isOpened() {
    return false;
}

bool PiCamera::read(cv::Mat &image) {
    return false;
}

bool PiCamera::set(int propId, double value) {
    return false;
}

#endif

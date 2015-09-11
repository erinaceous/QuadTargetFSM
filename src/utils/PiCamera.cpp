//
// Created by owain on 9/11/15.
//

#include "PiCamera.hpp"

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

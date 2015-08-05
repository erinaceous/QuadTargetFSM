//
// Created by owain on 03/08/15.
//

#include <opencv2/opencv.hpp>
#include "TargetFinder.h"

using namespace targetfinder;

cv::RotatedRect PersistentTarget::getRotatedRect() {
    return this->rotatedRect;
}

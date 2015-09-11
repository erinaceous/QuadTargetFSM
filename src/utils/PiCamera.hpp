//
// Created by owain on 9/11/15.
//

#ifndef QUADTARGETFSM_PICAMERA_HPP
#define QUADTARGETFSM_PICAMERA_HPP

#include <opencv2/opencv.hpp>

/**
 * Future class which will implement closer-to-the-metal communications with the
 * Raspberry Pi Camera using Broadcom's Multi-Media Access Layer, allowing for
 * stuff we can't get via the Video4Linux2 interface such as getting raw YUV420
 * frames, and setting most resolutions/target framerates.
 *
 * It may be better to change the main program flow when running on the
 * Pi so that we can make use of MMAL callbacks whenever a cam frame is
 * ready, so this will also implement two extra functions:
 *  void setOutput(Mat *ptr) -- where to store images when camera sends them;
 *  void setCallback(void *function) -- function to run when image is ready.
 */
class PiCamera : public cv::VideoCapture {
public:
    PiCamera(int idx);
    bool isOpened();
    bool set(int propId, double value);
    bool read(cv::Mat &image);
};


#endif //QUADTARGETFSM_PICAMERA_HPP

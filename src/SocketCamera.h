//
// Created by owain on 11/08/15.
//

#ifndef QUADTARGETFSM_SHMCAMERA_H
#define QUADTARGETFSM_SHMCAMERA_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <opencv2/opencv.hpp>


class SocketCamera : public cv::VideoCapture {
    public:
        SocketCamera(std::string addr);
        bool isOpened();
        bool set(int propId, double value);
        bool read(cv::Mat &image);
    private:
        std::string hostname;
        int width = 640, height = 480;
        int sockfd, portno = 5011;
        struct sockaddr_in serv_addr;
};


#endif //QUADTARGETFSM_SHMCAMERA_H

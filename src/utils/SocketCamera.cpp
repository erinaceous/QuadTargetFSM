//
// Created by owain on 11/08/15.
//

#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fstream>
#include <arpa/inet.h>
#include "SocketCamera.hpp"

SocketCamera::SocketCamera(std::string addr) {
    long port_ptr = addr.rfind(':');
    this->hostname = addr.substr(0, port_ptr);
    this->portno = atoi(addr.c_str() + port_ptr + 1);
    std::cout << "connecting to " << this->hostname << " on port " << this->portno << std::endl;
    memset(&this->serv_addr, 0, sizeof(this->serv_addr));
    this->sockfd = socket(AF_INET, SOCK_STREAM, 0);
    int on = 1;
    setsockopt(
            this->sockfd, SOL_SOCKET, SO_REUSEADDR,
            (const char*) &on, sizeof(on)
    );
    this->serv_addr.sin_family = AF_INET;
    this->serv_addr.sin_port = htons(this->portno);
    inet_pton(AF_INET, this->hostname.c_str(), &this->serv_addr.sin_addr);
    connect(this->sockfd, (sockaddr *) &this->serv_addr, sizeof(this->serv_addr));
}

bool SocketCamera::isOpened() {
    return true;
}

bool SocketCamera::set(int propId, float value) {
    switch(propId) {
        case CV_CAP_PROP_FRAME_WIDTH:
            this->width = (int) value;
            return true;
        case CV_CAP_PROP_FRAME_HEIGHT:
            this->height = (int) value;
            return true;
        default:
            return false;
    }
}

bool SocketCamera::read(cv::Mat &image) {
    long len = this->height * this->width * 3;
    char buf[len];
    send(this->sockfd, "OK", 2, MSG_NOSIGNAL);
    recv(this->sockfd, buf, len, MSG_WAITALL);
    char buf2[len];
    for(int i=0; i<len; i++) {
        buf2[i] = buf[len - (i + 1)];
    }
    cv::Mat mat(this->height, this->width, CV_8UC3, buf2, cv::Mat::AUTO_STEP);
    cv::flip(mat, mat, 1);
    mat.copyTo(image);
    return true;
}

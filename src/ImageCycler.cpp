//
// Created by owain on 8/20/15.
//

#include <dirent.h>
#include "ImageCycler.h"

ImageCycler::ImageCycler(std::string directory) {
    this->directory = directory;
    this->open(directory);
}

void ImageCycler::open(std::string directory) {
    this->idx = 0;
    this->files.clear();
    DIR* dirp = opendir(directory.c_str());
    bool addslash = directory.rfind('/') != directory.length() - 1;
    dirent* dp;
    while((dp = readdir(dirp)) != NULL) {
        char* extp = strrchr(dp->d_name, '.');
        // std::cout << extp << std::endl;
        if(strcasecmp(extp, ".jpg") == 0 || strcasecmp(extp, ".png") == 0 ||
           strcasecmp(extp, ".jpeg") == 0 || strcasecmp(extp, ".tif") == 0 ||
           strcasecmp(extp, ".gif") == 0 || strcasecmp(extp, ".bmp") == 0) {
            std::stringstream ss;
            ss << directory;
            if(addslash) {
                ss << "/";
            }
            ss << dp->d_name;
            this->files.push_back(ss.str());
        }
    }
}

bool ImageCycler::read(cv::Mat &image) {
    int64 current = cv::getTickCount();
    if(this->files.size() == 0) {
        return false;
    }
    if(this->idx >= this->files.size()) {
        return false;
    }
    if(this->fps == 0 || ((this->last_accessed + this->fps) <= current)) {
        std::string image_file = this->files[this->idx % this->files.size()];
        this->buffer = cv::imread(image_file, cv::IMREAD_COLOR);
        if(this->buffer.cols == 0) {
            return false;
        }
        if(this->width > 0 && this->height > 0) {
            cv::resize(this->buffer, this->buffer, cv::Size(this->width, this->height));
        }
        this->idx++;
        this->last_accessed = current;
    }
    this->buffer.copyTo(image);
    return true;
}

bool ImageCycler::set(int propId, double value) {
    switch(propId) {
        case CV_CAP_PROP_FRAME_WIDTH:
            this->width = (int) value;
            return true;
        case CV_CAP_PROP_FRAME_HEIGHT:
            this->height = (int) value;
            return true;
        case CV_CAP_PROP_FPS:
            this->fps = (int64) (cv::getTickFrequency() * (1.0 / value));
            return true;
        default:
            return false;
    }
}

std::string* ImageCycler::getFile() {
    return &this->files[(this->idx - 1) % this->files.size()];
}

bool ImageCycler::isOpened() {
    return true;
}

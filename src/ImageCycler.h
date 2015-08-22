//
// Created by owain on 8/20/15.
//

#ifndef QUADTARGETFSM_IMAGESEQUENCECAPTURE_H
#define QUADTARGETFSM_IMAGESEQUENCECAPTURE_H

#include <opencv2/opencv.hpp>

class ImageCycler : public cv::VideoCapture {
    public:
        static constexpr int FILENAME=0;

        ImageCycler(std::string directory);
        void open(std::string directory);
        bool read(cv::Mat &image);
        bool set(int propId, double value);
        bool isOpened();
        std::string* getFile();
    private:
        std::string directory;
        std::vector<std::string> files;
        unsigned long idx = 0;
        int width = 0, height = 0;
        int64 fps = 0;
        cv::Mat buffer;
        int64 last_accessed;
};

#endif //QUADTARGETFSM_IMAGESEQUENCECAPTURE_H

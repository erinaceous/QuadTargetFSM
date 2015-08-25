//
// Created by owain on 8/25/15.
//

#include <memory>
#include <opencv2/opencv.hpp>
#include <Utils.h>
#include "include/StateMachine.hpp"
#include "include/MarkerDetector.hpp"
#include "include/Marker.hpp"

namespace targetfinder {
    class CascadeDetector : public MarkerDetector {
    public:
        static constexpr int MIN_SIZE = 2;
        static constexpr int MIN_NEIGHBORS = 2;
        static constexpr double SCALE_FACTOR = 2.0;

        void setClassifier(std::string file) {
            this->classifier.load(file.c_str());
        }

        void setMinSize(int min_size) {
            this->min_size = min_size;
        }

        void setMinNeighbors(int min_neighbors) {
            this->min_neighbors = min_neighbors;
        }

        void setScaleFactor(double scale_factor) {
            this->scale_factor = scale_factor;
        }

        std::vector <std::shared_ptr<Marker>> detect(
                cv::Mat input, cv::Mat output, bool show_state
        ) {
            std::vector <std::shared_ptr<Marker>> markers;
            std::vector <cv::Rect> objects;

            this->classifier.detectMultiScale(
                    input, objects,
                    this->scale_factor,
                    this->min_neighbors,
                    0,
                    cv::Size(this->min_size, this->min_size)
            );

            for(int i=0; i<objects.size(); i++) {
                cv::Rect object = objects[i];
                Marker *m = new Marker(
                        object.x, object.x + object.width,
                        object.y, object.y + object.height,
                        0, 0, 0
                );
                markers.push_back(
                        std::shared_ptr<Marker>(m)
                );
            }

            return markers;
        };

    private:
        cv::CascadeClassifier classifier;
        int min_size = MIN_SIZE;
        int min_neighbors = MIN_NEIGHBORS;
        double scale_factor = SCALE_FACTOR;
    };
}

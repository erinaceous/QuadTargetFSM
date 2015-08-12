/*
 * Runs the target recognition algorithm on webcam feed or an image stream fed
 * from stdin.
 * Reports information on target position, orientation and scale as
 * comma-separated values (ASCII) in a specified UNIX socket.
 * This socket acts like a UDP one -- no buffering; if you don't read data when
 * it's available it won't keep it. Multiple programs can consume the data
 * (broadcast/multicast socket).
 */

#include <iostream>
#include <regex>
#include <opencv2/opencv.hpp>
#include "TargetFinder.h"
#include "SocketCamera.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

using namespace std;
using namespace cv;

static bool debug = false;
#define DEBUGPRINT(x) { if(debug) { std::cerr << x << std::endl; }}

int main(int argc, char* argv[]) {
    boost::property_tree::ptree pt;
    char* config_path = getenv("QUADTARGET_CONFIG");
    if(config_path == NULL) {
        boost::property_tree::ini_parser::read_ini("config.ini", pt);
    } else {
        boost::property_tree::ini_parser::read_ini(config_path, pt);
    }


    debug = pt.get<bool>("gui.debug");
    bool headless = pt.get<bool>("gui.headless");
    bool save_video = false;
    bool test_image = false;
    int wait_key = pt.get<int>("gui.waitKey");
    string video_file = pt.get<string>("camera.file");
    string save_video_file;
    if(argc == 2) {
        save_video_file = string(argv[1]);
    } else {
        save_video_file = pt.get<string>("gui.save_video");
    }
    int desired_fps = pt.get<int>("camera.fps");


    VideoCapture *cap;
    if(std::isdigit(video_file[0])) {
        cap = new VideoCapture(pt.get<int>("camera.file"));
    } else if(video_file.rfind(".png") != std::string::npos) {
        test_image = true;
    } else if(video_file.rfind(':') != std::string::npos) {
        cap = new SocketCamera(video_file);
    } else {
        cap = new VideoCapture(video_file);
    }
    if(!cap->isOpened()) {
        DEBUGPRINT("Couldn't open webcam or video");
        // return -1;
    }

    cap->set(CV_CAP_PROP_FRAME_WIDTH, pt.get<int>("camera.width"));
    cap->set(CV_CAP_PROP_FRAME_HEIGHT, pt.get<int>("camera.height"));
    cap->set(CV_CAP_PROP_FPS, desired_fps);
    cap->set(CV_CAP_PROP_CONVERT_RGB, pt.get<bool>("camera.convert_rgb"));

    cv::Mat *input = new cv::Mat();
    if(test_image) {
        Mat image = cv::imread(video_file);
        image.copyTo(*input);
    } else {
        cap->read(*input);
    }

    VideoWriter *wri = new VideoWriter();
    if(save_video_file.compare("none") != 0) {
        save_video = true;
        wri->open(save_video_file, CV_FOURCC('M', 'J', 'P', 'G'),
                 (desired_fps <= 0) ? 30 : desired_fps,  // set output FPS to 30 if input is "fast as possible"
                 cv::Size(input->cols, input->rows));
    }

    // cv::Mat processed;
    cv::Mat *output = new cv::Mat();
    targetfinder::TargetFinder *tf = new targetfinder::TargetFinder();
    targetfinder::PersistentTarget *target = new targetfinder::PersistentTarget();
    targetfinder::CameraModel *cm = new targetfinder::CameraModel(input->cols, input->rows);
    targetfinder::Navigator *nv = new targetfinder::Navigator(input->cols, input->rows);
    float target_influence = pt.get<float>("parameters.target_alpha");
    target->setTimeout(cv::getTickFrequency() * pt.get<float>("parameters.target_lifetime"));
    target->setAngleOffset(pt.get<float>("camera.angle_offset"));
    tf->setRowStep(pt.get<int>("parameters.row_step"));
    tf->setMinLength(pt.get<int>("parameters.min_length"));
    tf->setTolerance(pt.get<float>("parameters.tolerance"));
    tf->setNumBins(pt.get<int>("parameters.num_bins"));
    tf->setMarkerAspectTolerance(pt.get<float>("parameters.marker_aspect_tolerance"));
    tf->setMarkerDistances(
            pt.get<float>("parameters.min_marker_distance"),
            pt.get<float>("parameters.max_marker_distance")
    );
    tf->setMarkerSizeTolerance(pt.get<float>("parameters.marker_size_tolerance"));
    tf->setAngleOffset(pt.get<float>("camera.angle_offset"));
    bool show_state = pt.get<bool>("gui.show_state");
    bool convert_yuv = pt.get<bool>("camera.convert_yuv");
    bool flip_vertical = pt.get<bool>("camera.flip_vertical");
    bool flip_horizontal = pt.get<bool>("camera.flip_horizontal");
    bool running = true;
    int64 start = cv::getTickCount();
    int64 current;
    float fps = 1.0;
    static constexpr float alphafps = 0.05;
    cv::Point img_center(input->cols / 2, input->rows / 2);
    while(running) {
        current = cv::getTickCount();
        if (!test_image) {
            if(!cap->read(*input)) {
                DEBUGPRINT("couldn't get video frame");
                cv::waitKey(1000);
                continue;
            }
        }

        if(flip_vertical && flip_horizontal) {
            cv::flip(*input, *input, -1);
        } else if(flip_vertical && !flip_horizontal) {
            cv::flip(*input, *input, 0);
        } else if(flip_horizontal && !flip_vertical) {
            cv::flip(*input, *input, 1);
        }

        if(!headless || save_video) {
            input->copyTo(*output);
        }

        if(convert_yuv) {
            cv::cvtColor(*input, *input, cv::COLOR_BGR2YUV);
        }

        std::vector<targetfinder::Target> targets = tf->doTargetRecognition(*input, *output, show_state);

        targetfinder::Target *best_target = nullptr;
        cv::Size best_target_size = cv::Size(0, 0);
        for(int i=0; i<targets.size(); i++) {
            if(targets[i].calc_valid) {
                cv::Size target_size = targets[i].rect().size();
                if (target_size.width >= best_target_size.width
                    && target_size.height >= best_target_size.height) {
                    best_target_size = target_size;
                    best_target = &targets[i];
                }
            }
        }
        std::cout << "{\"target\": ";
        if(best_target) {
            target->update(best_target, current, target_influence);
        }
        if(target->alive(current)) {
            cv::Rect r = target->rect();
            cv::Point center = target->center();
            float real_distance = cm->distance(r.width, r.height) * 0.001;
            float angle = target->angle();
            nv->update(center, angle, real_distance, target->age());
            std::cout << "{" << target->str();
            std::cout << ", \"distance(m)\": " << real_distance << "}";
            if (!headless || save_video) {
                cv::Scalar c_black = cv::Scalar(0, 0, 0);
                cv::Scalar c_red = cv::Scalar(0, 0, 255);
                cv::Scalar c_green = cv::Scalar(0, 255, 0);
                cv::rectangle(*output, r, c_black, 3);
                cv::rectangle(*output, r, c_red, 1);
                cv::circle(*output, center, 5, c_black, -1);
                cv::Point endPoint = cv::Point(
                        center.x + r.width * cos(angle),
                        center.y + r.height * sin(angle)
                );
                cv::line(*output, center, endPoint, c_black, 3);
                cv::line(*output, center, endPoint, c_red, 1);
                cv::circle(*output, center, 3, c_red, -1);
                const char *distanceStr = "%0.2f m";
                cv::putText(*output, format(distanceStr, real_distance), center,
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, c_black, 4);
                cv::putText(*output, format(distanceStr, real_distance), center,
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, c_green, 2);
            }
        } else {
            std::cout << "null";
            nv->update();
        }
        std::cout << ", \"sticks\": {" << nv->str() << "}";
        std::cout << ", \"fps\": "<< fps << ", \"time\": " << current << "}" << std::endl;

        int64 ticks = cv::getTickCount() - current;
        float rate = 1 / (ticks / cv::getTickFrequency());
        fps = (alphafps * rate) + (1.0 - alphafps) * fps;

        if(!headless || save_video) {
            // cv::imshow("input", input);
            const char *tickStr = "fps = %0.0f";
            cv::putText(*output, format(tickStr, fps), cv::Point(10, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
            cv::putText(*output, format(tickStr, fps), cv::Point(10, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
            int ll = 150;
            cv::line(*output, cv::Point(img_center.x, img_center.y - ll), cv::Point(img_center.x, img_center.y + ll),
                     cv::Scalar(0, 0, 0), 3);
            cv::line(*output, cv::Point(img_center.x - ll, img_center.y), cv::Point(img_center.x + ll, img_center.y),
                     cv::Scalar(0, 0, 0), 3);
            cv::line(*output, cv::Point(img_center.x, img_center.y - ll), cv::Point(img_center.x, img_center.y + ll),
                     cv::Scalar(255, 255, 255), 1);
            cv::line(*output, cv::Point(img_center.x - ll, img_center.y), cv::Point(img_center.x + ll, img_center.y),
                     cv::Scalar(255, 255, 255), 1);
            cv::circle(*output, nv->image_point(0, ll), 5, cv::Scalar(0, 0, 0), -1);
            cv::circle(*output, nv->image_point(0, ll), 3, cv::Scalar(0, 255, 0), -1);
            cv::circle(*output, nv->image_point(1, ll), 5, cv::Scalar(0, 0, 0), -1);
            cv::circle(*output, nv->image_point(1, ll), 3, cv::Scalar(0, 255, 0), -1);
        }
        if(save_video) {
            wri->write(*output);
        }
        if(!headless) {
            cv::imshow("output", *output);
            cv::waitKey(wait_key);
        }
    }

    return 0;
}
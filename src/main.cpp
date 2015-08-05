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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

using namespace std;
using namespace cv;

bool debug = false;
#define DEBUGPRINT(x) { if(debug) { std::cerr << x << std::endl; }}

int main() {
    boost::property_tree::ptree pt;
    char* config_path = getenv("QUADTARGET_CONFIG");
    if(config_path == NULL) {
        boost::property_tree::ini_parser::read_ini("config.ini", pt);
    } else {
        boost::property_tree::ini_parser::read_ini(config_path, pt);
    }


    debug = pt.get<bool>("gui.debug");
    bool headless = pt.get<bool>("gui.headless");
    bool loop_video = false;
    bool save_video = false;
    bool test_image = false;
    int wait_key = pt.get<int>("gui.waitKey");
    string video_file = pt.get<string>("camera.file");
    string save_video_file = pt.get<string>("gui.save_video");


    VideoCapture cap;
    if(std::isdigit(video_file[0])) {
        cap = VideoCapture(pt.get<int>("camera.file"));
    } else if(video_file.rfind(".png") != std::string::npos) {
        test_image = true;
    } else {
        cap = VideoCapture(video_file);
        loop_video = true;
    }
    if(!cap.isOpened()) {
        cerr << "Couldn't open webcam or video" << endl;
        // return -1;
    }
    cap.set(CV_CAP_PROP_FRAME_WIDTH, pt.get<int>("camera.width"));
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, pt.get<int>("camera.height"));
    cap.set(CV_CAP_PROP_FPS, pt.get<int>("camera.fps"));
    // cap.set(CV_CAP_PROP_CONVERT_RGB, false);

    cv::Mat input;
    if(test_image) {
        input = cv::imread(video_file);
    } else {
        cap.read(input);
    }

    VideoWriter wri;
    if(save_video_file.compare("none") != 0) {
        save_video = true;
        wri.open(save_video_file, CV_FOURCC('M', 'J', 'P', 'G'), 30,
                 cv::Size(input.cols, input.rows));
    }

    cv::Mat processed;
    cv::Mat output;
    targetfinder::TargetFinder tf;
    tf.setMinLength(pt.get<int>("parameters.min_length"));
    tf.setTolerance(pt.get<float>("parameters.tolerance"));
    tf.setNumBins(pt.get<int>("parameters.num_bins"));
    tf.setMarkerAspectTolerance(pt.get<float>("parameters.marker_aspect_tolerance"));
    tf.setMarkerDistances(
            pt.get<float>("parameters.min_marker_distance"),
            pt.get<float>("parameters.max_marker_distance")
    );
    tf.setMarkerSizeTolerance(pt.get<float>("parameters.marker_size_tolerance"));
    bool show_state = pt.get<bool>("gui.show_state");

    bool running = true;
    int64 start = cv::getTickCount();
    int64 current;
    float fps = 1.0;
    static constexpr float alphafps = 0.05;
    while(running) {
        current = cv::getTickCount();
        if (!test_image) {
            if(!cap.read(input)) {
                DEBUGPRINT("couldn't get video frame");
                cv::waitKey(1000);
                continue;
            }
        }

        if(!headless || save_video) {
            input.copyTo(output);
        }

        cv::cvtColor(input, processed, cv::COLOR_BGR2YUV);
        std::vector<targetfinder::Target> targets = tf.doTargetRecognition(processed, output, show_state);

        for(int i=0; i<targets.size(); i++) {
            if(targets[i].calc_valid) {
                std::cout << i << ": " << targets[i].str() << ", fps: " << fps << std::endl;
                if(!headless || save_video) {
                    cv::RotatedRect r = targets[i].rotatedRect();
                    cv::ellipse(output, r, cv::Scalar(0, 0, 0), 3);
                    cv::ellipse(output, r, cv::Scalar(0, 0, 255), 1);
                    cv::circle(output, r.center, 5, cv::Scalar(0, 0, 0), -1);
                    cv::circle(output, r.center, 3, cv::Scalar(0, 0, 255), -1);
                    cv::Point endPoint = cv::Point(
                            r.center.x + r.size.width * cos(r.angle),
                            r.center.y + r.size.height * sin(r.angle)
                    );
                    cv::line(output, r.center, endPoint, cv::Scalar(0, 0, 0), 3);
                    cv::line(output, r.center, endPoint, cv::Scalar(0, 0, 255), 1);
                }
            }
        }

        std::vector<targetfinder::PersistentTarget> persistentTargets = tf.getPersistentTargets();
        for(int i=0; i<persistentTargets.size(); i++) {
            if(!headless) {
                cv::RotatedRect r = persistentTargets[i].getRotatedRect();
                cv::ellipse(output, r, cv::Scalar(0, 0, 0), 3);
                cv::ellipse(output, r, cv::Scalar(0, 255, 0), 1);
            }
        }

        int64 ticks = cv::getTickCount() - current;
        float rate = 1 / (ticks / cv::getTickFrequency());
        fps = (alphafps * rate) + (1.0 - alphafps) * fps;

        if(!headless || save_video) {
            // cv::imshow("input", input);
            const char *tickStr = "fps = %0.0f";
            cv::putText(output, format(tickStr, fps), cv::Point(10, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
            cv::putText(output, format(tickStr, fps), cv::Point(10, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
        }
        if(!headless) {
            cv::imshow("output", output);
        }
        if(save_video) {
            wri.write(output);
        }
        cv::waitKey(wait_key);
    }

    return 0;
}
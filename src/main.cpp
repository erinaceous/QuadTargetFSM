/*
 * Runs the target recognition algorithm on a video stream.
 * Reports information on target position, orientation and scale as well as
 * pitch/roll/yaw/throttle requests, in a JSON structure, for every video frame
 * processed. Outputs this directly to stdout to be read by other programs.
 *
 * Original Author: Owain Jones [odj@aber.ac.uk]
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "include/MarkerDetector.hpp"
#include "FSMDetector.cpp"
#include "CascadeDetector.cpp"
#include "include/Target.hpp"
#include "include/PersistentTarget.hpp"
#include "include/CameraModel.hpp"
#include "include/TargetFinder.hpp"
#include "utils/SocketCamera.hpp"
#include "utils/ImageCycler.hpp"
#include "include/Navigator.hpp"
#include "utils/ConfigMerger.hpp"

using namespace std;
using namespace cv;

static bool debug = false;
#define DEBUGPRINT(x) { if(debug) { std::cerr << x << std::endl; }}

int main(int argc, char* argv[]) {
    // TODO: This is becoming a very long function. Split it up a bit.
    /*
     * Parse config files, load their contents into a bunch of local
     * variables.
     * The config files loaded later override values in earlier files.
     */
    boost::property_tree::ptree pt;
    boost::property_tree::ptree pt_updates;
    string config_paths = getenv("QUADTARGET_CONFIGS");
    if(config_paths.empty()) {
        boost::property_tree::ini_parser::read_ini("config.ini", pt);
    } else {
        std::vector<std::string> config_strs;
        boost::split(config_strs, config_paths, boost::is_any_of(" ,;:"));
        for(int i=0; i<config_strs.size(); i++) {
            if(config_strs[i].empty()) {
                continue;
            }
            std::cerr << "Reading config from " << config_strs[i] << std::endl;
            boost::property_tree::ini_parser::read_ini(config_strs[i].c_str(), pt_updates);
            pt = ConfigMerger::mergePropertyTrees(pt, pt_updates);
        }
    }
    debug = pt.get<bool>("gui.debug");
    bool headless = pt.get<bool>("gui.headless");
    bool save_video = false;
    bool test_image = false;
    bool endless_video = false;
    int wait_key = pt.get<int>("gui.waitKey");
    string video_file = pt.get<string>("camera.file");
    string save_video_file;
    if(argc == 2) {
        save_video_file = string(argv[1]);
    } else if(argc == 3) {
        save_video_file = string(argv[1]);
        video_file = string(argv[2]);
    } else {
        save_video_file = pt.get<string>("gui.save_video");
    }
    bool save_snapshots = false;
    string save_snapshot_path = pt.get<string>("gui.save_snapshots");
    if(save_snapshot_path.compare("none") != 0) {
        save_snapshots = true;
    }
    unsigned long target_count = 0;
    double desired_fps = pt.get<double>("camera.fps");
    bool render_input = pt.get<bool>("render.input");
    bool render_fps = pt.get<bool>("render.fps");
    bool render_navigator = pt.get<bool>("render.navigator");
    bool render_target = pt.get<bool>("render.target");
    bool render_markers = pt.get<bool>("render.markers");
    bool render_fsm = pt.get<bool>("render.fsm_state");
    bool output_target = pt.get<bool>("output.target");
    bool output_navigator = pt.get<bool>("output.navigator");
    bool output_times = pt.get<bool>("output.times");
    std::string *marker_info = nullptr;
    bool output_marker_info = pt.get<bool>("output.marker_info");
    int scale_w = pt.get<int>("gui.scale_w");
    int scale_h = pt.get<int>("gui.scale_h");

    /*
     * Create our video input.
     * If the video_file config item starts with a digit, we assume it's
     * a V4L2 camera index (usually 0).
     * If it's a string ending in .png, we assume it's a single test image,
     * which we will load once.
     * If it has a colon in it, we assume it's a URL e.g. localhost:5011,
     * so we create a SocketCamera.
     * Otherwise, it's treated as a video file.
     */
    VideoCapture *cap;
    if(std::isdigit(video_file[0])) {
        cap = new VideoCapture(pt.get<int>("camera.file"));
        endless_video = true;
    } else if(video_file.rfind(".png") != std::string::npos) {
        test_image = true;
        endless_video = true;
    } else if(video_file.rfind("/") == video_file.length() - 1) {
        cap = new ImageCycler(video_file);
        endless_video = true;
    } else if(video_file.rfind(':') != std::string::npos) {
        cap = new SocketCamera(video_file);
        endless_video = true;
    } else {
        cap = new VideoCapture(video_file);
    }
    if(!test_image) {
        if (!cap->isOpened()) {
            DEBUGPRINT("Couldn't open webcam or video");
            // return -1;
        }
    }

    /*
     * If possible, set properties for the video input. These really only apply
     * to the live webcam input and are ignored for the others, although
     * SocketCamera needs these to be correct.
     */
    if(!test_image) {
        cap->set(CV_CAP_PROP_FRAME_WIDTH, pt.get<int>("camera.width"));
        cap->set(CV_CAP_PROP_FRAME_HEIGHT, pt.get<int>("camera.height"));
        cap->set(CV_CAP_PROP_FPS, desired_fps);
        cap->set(CV_CAP_PROP_CONVERT_RGB, pt.get<bool>("camera.convert_rgb"));
    }

    cv::Mat *input = new cv::Mat();
    if(test_image) {
        Mat image = cv::imread(video_file);
        image.copyTo(*input);
    } else {
        cap->read(*input);
    }

    if(scale_w == 0 || scale_h == 0) {
        scale_w = input->cols;
        scale_h = input->rows;
    }

    /*
     * If the save_video config entry is something other than "none", create
     * a video encoding instance using the MJPG format.
     * This uses ffmpeg under the hood and technically supports any FOURCC codes
     * that ffmpeg does, but MJPG has a good performance/size tradeoff and works
     * very well on the Pi, in lieu of being able to easily use the hardware
     * h.264 encoder.
     */
    VideoWriter *wri = new VideoWriter();
    if(save_video_file.compare("none") != 0) {
        save_video = true;
        int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
        wri->open(save_video_file, fourcc,
                 (desired_fps <= 0) ? 30 : desired_fps,  // set output FPS to 30 if input is "fast as possible"
                 cv::Size(scale_w, scale_h));
    }

    cv::Mat output(input->rows, input->cols, CV_8UC3);

    /*
     * Create our marker detection, target finder, target, camera model and
     * navigator instances...
     */
    targetfinder::MarkerDetector *detector;
    targetfinder::TargetFinder *tf = new targetfinder::TargetFinder();
    targetfinder::PersistentTarget *target = new targetfinder::PersistentTarget();
    targetfinder::CameraModel *cm = new targetfinder::CameraModel(input->cols, input->rows);
    targetfinder::Navigator *nv = new targetfinder::Navigator(input->cols, input->rows);

    /*
     * ...and pass them a bunch of configuration stuff from the .ini files.
     */
    if(pt.get<std::string>("parameters.method").compare("cascade") == 0) {
        detector = new targetfinder::CascadeDetector();
        #define cas ((targetfinder::CascadeDetector *)detector)
        cas->setClassifier(pt.get<std::string>("parameters.cascade.classifier_file"));
        cas->setMinSize(pt.get<int>("parameters.cascade.min_size"));
        cas->setMinNeighbors(pt.get<int>("parameters.cascade.min_neighbors"));
        cas->setScaleFactor(pt.get<double>("parameters.cascade.scale_factor"));
    } else {
        detector = new targetfinder::FSMDetector();
        #define fsm ((targetfinder::FSMDetector *) detector)
        fsm->setRowStep(pt.get<int>("parameters.fsm.row_step"));
        fsm->setMinThreshold(pt.get<int>("parameters.fsm.min_threshold"));
        fsm->setMinLength(pt.get<int>("parameters.fsm.min_length"));
        fsm->setSmoothness(pt.get<double>("parameters.fsm.threshold_smoothness"));
        fsm->setTolerance(pt.get<double>("parameters.fsm.tolerance"));
        fsm->setMarkerAspectTolerance(pt.get<double>("parameters.fsm.marker_aspect_tolerance"));
    }
    nv->setPIDs(
            pt.get<double>("navigator.pitch_p"),
            pt.get<double>("navigator.pitch_i"),
            pt.get<double>("navigator.pitch_d"),
            pt.get<double>("navigator.roll_p"),
            pt.get<double>("navigator.roll_i"),
            pt.get<double>("navigator.roll_d")
    );
    target->setTimeout(cv::getTickFrequency() * pt.get<double>("target.lifetime"));
    target->setAngleOffset(pt.get<double>("camera.angle_offset"));
    tf->setMarkerDistances(
            pt.get<double>("target.min_marker_distance"),
            pt.get<double>("target.max_marker_distance")
    );
    tf->setMarkerSizeTolerance(pt.get<double>("target.marker_size_tolerance"));

    /*
     * Some more miscellaneous config stuff loaded into local variables
     */
    double target_influence = pt.get<double>("target.alpha");
    int min_age = pt.get<int>("target.min_age");
    bool convert_yuv = pt.get<bool>("camera.convert_yuv");
    bool flip_vertical = pt.get<bool>("camera.flip_vertical");
    bool flip_horizontal = pt.get<bool>("camera.flip_horizontal");
    bool wait_for_input = pt.get<bool>("gui.wait_for_input");

    bool running = true;
    int64 start = cv::getTickCount();
    int64 t_start, t_framegrab, t_preprocessing, t_detect, t_group, t_select,
            t_render, t_output, t_end;
    double fps = 1.0, delta = 0.0, hz = 1.0;
    static constexpr double alphafps = 0.05;
    cv::Point img_center(input->cols / 2, input->rows / 2);

    cv::Scalar c_black = cv::Scalar(0, 0, 0);
    cv::Scalar c_red = cv::Scalar(0, 0, 255);
    cv::Scalar c_green = cv::Scalar(0, 255, 0);
    cv::Scalar c_yellow(0, 255, 255);

    /*
     * Main loop
     */
    while(running) {
        if(wait_for_input) {
            getchar();
        }
        t_start = cv::getTickCount();
        if (!test_image) {
            if(!cap->read(*input)) {
                DEBUGPRINT("couldn't get video frame");
                cv::waitKey(1000);
                if(endless_video) {
                    continue;
                } else {
                    return 0;
                }
            }
        }
        t_framegrab = cv::getTickCount();

        if(output_marker_info && !test_image) {
            marker_info = ((ImageCycler*)cap)->getFile();
        }

        /*
         * Flip the image vertically and horizontally if needed
         */
        if(flip_vertical && flip_horizontal) {
            cv::flip(*input, *input, -1);
        } else if(flip_vertical && !flip_horizontal) {
            cv::flip(*input, *input, 0);
        } else if(flip_horizontal && !flip_vertical) {
            cv::flip(*input, *input, 1);
        }

        /*
         * Determine whether the output image background is a copy of the input
         * image or whether we just leave it as black
         */
        if((!headless || save_video) && render_input) {
            input->copyTo(output);
        } else {
            output.zeros(output.rows, output.cols, CV_8UC3);
        }

        /*
         * Converting to YUV means we can efficiently access the pixel intensity
         * as channel 0 and the colour levels as channels 1 and 2
         *  y = image[(y * width) + (x * 3)], u = image[... + 1], ...
         * Most of the target finding stuff operates on a grayscale image (Y'),
         * but the colour (U and V) channels come in handy for calculating
         * local/global pixel variances.
         * Y'UV is picked over e.g. HSV/HSL/L*ab colour spaces because most
         * webcams natively support outputting YCrCb images -- so potentially
         * we could shave off a few CPU cycles in future by getting a YUV image
         * directly from the Pi MMAL library rather than going
         *      pi cam -> v4l: yuv to rgb -> opencv: rgb to yuv
         */
        if(convert_yuv) {
            cv::cvtColor(*input, *input, cv::COLOR_BGR2YUV);
        }

        t_preprocessing = cv::getTickCount();

        /*
         * Run the algorithm for finding potential target markers in the input
         * image.
         */
        std::vector<std::shared_ptr<targetfinder::Marker>> markers = detector->detect(
                *input, output, render_fsm
        );
        t_detect = cv::getTickCount();

        /*
         * This returns a list of potential targets -- regardless of
         * whether they're valid or not.
         * Valid ones are .calc_valid == true.
         * (calc_valid is set when running the Target::valid() function, but
         * this is called inside the target recognition algorithm already)
         */
        std::vector<targetfinder::Target> targets = tf->groupTargets(
                *input, output, markers,
                render_markers, marker_info
        );
        t_group = cv::getTickCount();

        /*
         * Now we (possibly) have a list of targets, we need to select the best
         * one.
         * To do this, we calculate the euclidean distance between each target
         * and the existing "persistent target". The one with the smallest
         * distance is chosen as the best candidate target. Only valid targets
         * are considered.
         */
        targetfinder::Target *best_target = nullptr;
        double best_similarity = -1.0;
        for(int i=0; i<targets.size(); i++) {
            if(!targets[i].calc_valid) {
                continue;
            }
            double cur_similarity = target->similarity(&targets[i]);
            if(best_similarity == -1.0 || cur_similarity < best_similarity) {
                best_similarity = cur_similarity;
                best_target = &targets[i];
            }
        }
        t_select = cv::getTickCount();

        /*
         * Show which marker is the corner of the target, for debug purposes
         */
        if(render_target) {
            if(best_target) {
                std::shared_ptr<targetfinder::Marker> *this_markers = best_target->getMarkers();
                for(int i=0; i<3; i++) {
                    cv::Point this_center = this_markers[i]->center();
                    const char *markerStr = "%d";
                    cv::putText(output, format(markerStr, i), this_center,
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, c_black, 4);
                    cv::putText(output, format(markerStr, i), this_center,
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, c_yellow, 2);
                }

                std::shared_ptr<targetfinder::Marker> best_corner = best_target->getCorner();
                cv::Rect corner_rect = best_corner->rect();
                cv::rectangle(output, corner_rect, cv::Scalar(0, 255, 255), 2);
            }
        }

        /*
         * If we have found a good target, we update the existing target with
         * the x,y,width,height and angle of the new target.
         * target_influence adjusts how "big" the update is.
         */
        if(best_target) {
            target->update(best_target, t_start, target_influence);
        } else {
            target->tick();
        }

        if(output_target) {
            std::cout << "{\"target\": ";
        }

        /*
         * If the target is "alive" (0 <= target age < maximum lifetime), we
         * send its information to the Navigator instance, as well as output the
         * target info. in JSON format.
         */
        if(target->alive(t_start) && target->age() >= min_age) {
            cv::Rect r = target->rect();
            cv::Point center = target->center();
            double velocity = target->velocity();
            double real_distance = cm->distance(r.width, r.height) * 0.001;
            double angle = target->angle();
            if(best_similarity != 0.0) {
                nv->update(center, angle, real_distance, target->age(), velocity);
            } else {
                nv->update(center, angle, real_distance, target->age(), velocity);
            }
            if(output_target) {
                std::cout << "{" << target->str();
                std::cout << ", \"distance(m)\": " << real_distance << "}";
            }
            if ((!headless || save_video) && render_target) {
                cv::rectangle(output, r, c_black, 3);
                cv::rectangle(output, r, c_red, 1);
                cv::circle(output, center, 5, c_black, -1);
                cv::Point endPoint = cv::Point(
                        center.x + r.width * cos(angle),
                        center.y + r.height * sin(angle)
                );
                cv::line(output, center, endPoint, c_black, 3);
                cv::line(output, center, endPoint, c_red, 1);
                cv::circle(output, center, 3, c_red, -1);
                const char *distanceStr = "%0.2f m";
                cv::putText(output, format(distanceStr, real_distance), center,
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, c_black, 4);
                cv::putText(output, format(distanceStr, real_distance), center,
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, c_green, 2);
            }
        } else {
            /*
             * If the target is dead, we ask the Navigator to return to center.
             */
            if(output_target) {
                std::cout << "null";
            }
            nv->update(delta);
        }

        if(output_target && output_navigator) {
            std::cout << ", \"sticks\": {" << nv->str() << "}";
        } if(output_target) {
            std::cout << ", \"fps\": " << fps << ", \"time\": " << t_start <<
                         "}" << std::endl;
        }

        /*
         * If we are writing a video or outputting to screen, render some
         * more stuff to the output image -- FPS in top left corner,
         * navigator represented as a crosshair with dots showing the pitch/roll
         * values.
         */
        if(!headless || save_video) {
            if(render_fps) {
                const char *tickStr = "fps = %0.0f";
                std::string tickStrFormatted = format(tickStr, fps);
                cv::Point fpsPos(10, 20);
                cv::putText(output, tickStrFormatted, fpsPos,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, c_black,
                            3);
                cv::putText(output, tickStrFormatted, fpsPos,
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            c_yellow);
            }
            if(render_navigator) {
                int ll = 150;
                cv::line(output, cv::Point(img_center.x, img_center.y - ll),
                         cv::Point(img_center.x, img_center.y + ll),
                         cv::Scalar(0, 0, 0), 3);
                cv::line(output, cv::Point(img_center.x - ll, img_center.y),
                         cv::Point(img_center.x + ll, img_center.y),
                         cv::Scalar(0, 0, 0), 3);
                cv::line(output, cv::Point(img_center.x, img_center.y - ll),
                         cv::Point(img_center.x, img_center.y + ll),
                         cv::Scalar(255, 255, 255), 1);
                cv::line(output, cv::Point(img_center.x - ll, img_center.y),
                         cv::Point(img_center.x + ll, img_center.y),
                         cv::Scalar(255, 255, 255), 1);
                cv::circle(output, nv->image_point(0, ll), 5,
                           cv::Scalar(0, 0, 0), -1);
                cv::circle(output, nv->image_point(0, ll), 3,
                           cv::Scalar(0, 255, 0), -1);
                cv::circle(output, nv->image_point(1, ll), 5,
                           cv::Scalar(0, 0, 0), -1);
                cv::circle(output, nv->image_point(1, ll), 3,
                           cv::Scalar(0, 255, 0), -1);
            }
        }
        if(!headless || save_video && (scale_w > 0 && scale_h > 0)) {
            cv::resize(output, output, cv::Size(scale_w, scale_h));
        }
        t_render = cv::getTickCount();

        if(save_video) {
            wri->write(output);
        }
        if(save_snapshots && target->alive(t_start) && target->age() <= 1) {
            cv::imwrite(format(save_snapshot_path.c_str(), target_count++), output);
        }
        if(!headless) {
            cv::imshow("output", output);
            cv::waitKey(wait_key);
        }
        t_output = cv::getTickCount();

        // Output times spent on each part of the loop.
        if(output_times) {
            double tickFreq = cv::getTickFrequency();
            std::cout << "framegrab=" << ((t_framegrab - t_start) / tickFreq);
            std::cout << ", preprocess=" << ((t_preprocessing - t_framegrab) / tickFreq);
            std::cout << ", detect=" << ((t_detect - t_preprocessing) / tickFreq);
            std::cout << ", group=" << ((t_group - t_detect) / tickFreq);
            std::cout << ", select=" << ((t_select - t_group) / tickFreq);
            std::cout << ", render=" << ((t_render - t_select) / tickFreq);
            std::cout << ", output=" << ((t_output - t_render) / tickFreq);
            std::cout << std::endl;
        }

        // Calculate overall framerate.
        t_end = cv::getTickCount();
        delta = (t_end - t_start) / cv::getTickFrequency();
        hz = 1.0 / delta;
        fps += alphafps * (hz - fps);

    }

    return 0;
}

/*
 * Runs the target recognition algorithm on a video stream.
 * Reports information on target position, orientation and scale as well as
 * pitch/roll/yaw/throttle requests, in a JSON structure, for every video frame
 * processed. Outputs this directly to stdout to be read by other programs.
 *
 * Original Author: Owain Jones [odj@aber.ac.uk]
 */

#ifdef PI_BUILD
    #define pi_build true
#else
    #define pi_build false
#endif

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
#include "utils/PiCamera.hpp"
#include "include/Navigator.hpp"
#include "utils/ConfigMerger.hpp"

using namespace std;
using namespace cv;
using namespace targetfinder;

static bool debug = false;
#define DEBUGPRINT(x) { if(debug) { cerr << x << endl; }}

static constexpr double default_fps = 30;

// Define some colours for debug output
static const Scalar c_black = Scalar(0, 0, 0);
static const Scalar c_white = Scalar(255, 255, 255);
static const Scalar c_red = Scalar(0, 0, 255);
static const Scalar c_green = Scalar(0, 255, 0);
static const Scalar c_yellow(0, 255, 255);

// Define all our config variables and pointers to objects initialized later
bool headless, save_video, test_image, endless_video, save_snapshots,
        render_input, render_detector, render_markers, render_target,
        render_navigator, render_fps, output_target, output_navigator,
        output_times, output_marker_info, output_fsm_states,
        convert_yuv, flip_vertical, flip_horizontal, wait_for_input, running,
        async_mode;
unsigned short target_count;
int wait_key, scale_w, scale_h, min_age;
double desired_fps, target_influence, alphafps = 0.05, fps, fps_delta, fps_hz;
string video_file, save_video_file, save_snapshot_path, name_str_formatted;
string *marker_info = nullptr;
boost::property_tree::ptree pt;
VideoCapture *cap;
VideoWriter *wri;
MarkerDetector *detector;
TargetFinder *tf;
PersistentTarget *target;
CameraModel *cm;
Navigator *nv;
Mat *input;
Mat output;
Point img_center;
int64 t_start, t_framegrab, t_preprocessing, t_detect, t_group, t_select,
        t_render, t_output, t_end;


void init(int argc, char* argv[]) {
    /*
     * Parse config files, load their contents into a bunch of local
     * variables.
     * The config files loaded later override values in earlier files.
    */
    boost::property_tree::ptree pt_updates;
    string config_paths = getenv("QUADTARGET_CONFIGS");
    if(config_paths.empty()) {
        boost::property_tree::ini_parser::read_ini("config.ini", pt);
    } else {
        vector<string> config_strs;
        boost::split(config_strs, config_paths, boost::is_any_of(" ,;:"));
        for(int i=0; i<config_strs.size(); i++) {
            if(config_strs[i].empty()) {
                continue;
            }
            cerr << "Reading config from " << config_strs[i] << endl;
            boost::property_tree::ini_parser::read_ini(config_strs[i].c_str(), pt_updates);
            pt = ConfigMerger::mergePropertyTrees(pt, pt_updates);
        }
    }
    debug = pt.get<bool>("gui.debug");
    headless = pt.get<bool>("gui.headless");
    save_video = false;
    test_image = false;
    endless_video = false;
    wait_key = pt.get<int>("gui.waitKey");
    video_file = pt.get<string>("camera.file");
    if(argc == 2) {
        save_video_file = string(argv[1]);
    } else if(argc == 3) {
        save_video_file = string(argv[1]);
        video_file = string(argv[2]);
    } else {
        save_video_file = pt.get<string>("gui.save_video");
    }
    save_snapshots = false;
    save_snapshot_path = pt.get<string>("gui.save_snapshots");
    if(save_snapshot_path.compare("none") != 0) {
        save_snapshots = true;
    }
    target_count = 0;
    desired_fps = pt.get<double>("camera.fps");
    render_input = pt.get<bool>("render.input");
    render_fps = pt.get<bool>("render.fps");
    render_navigator = pt.get<bool>("render.navigator");
    render_target = pt.get<bool>("render.target");
    render_markers = pt.get<bool>("render.markers");
    render_detector = pt.get<bool>("render.detector");
    output_target = pt.get<bool>("output.target");
    output_navigator = pt.get<bool>("output.navigator");
    output_times = pt.get<bool>("output.times");
    marker_info = nullptr;
    output_marker_info = pt.get<bool>("output.marker_info");
    output_fsm_states = pt.get<bool>("output.fsm_states");
    scale_w = pt.get<int>("gui.scale_w");
    scale_h = pt.get<int>("gui.scale_h");

    /*
     * Create our video input.
     */
    // Check for V4L2 camera index
    if(video_file.find("v4l2:") != string::npos) {
        string cam_idx = video_file.substr(video_file.find(':'),
                                           video_file.length());
        cap = new VideoCapture(atoi(cam_idx.c_str()));
        endless_video = true;
    // Check for Pi Camera index
    } else if(video_file.find("pi:") != string::npos) {
        string cam_idx = video_file.substr(video_file.find(':'),
                                           video_file.length());
        cap = new PiCamera(atoi(cam_idx.c_str()));
        endless_video = true;
        async_mode = true;
    // Single static .png image
    } else if(video_file.rfind(".png") != string::npos) {
        test_image = true;
        endless_video = true;
    // Directory containing sequence of images
    } else if(video_file.rfind("/") == video_file.length() - 1) {
        cap = new ImageCycler(video_file);
        endless_video = true;
    // hostname:port pair, create a socket camera input
    } else if(video_file.rfind(':') != string::npos) {
        cap = new SocketCamera(video_file);
        endless_video = true;
    // video input from file
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
     * (There is no real protocol for SocketCamera, it sends an 'OK' and then
     * reads width*height*3 bytes from the socket and assumes that's the
     * whole image frame)
     */
    if(!test_image) {
        cap->set(CV_CAP_PROP_FRAME_WIDTH, pt.get<int>("camera.width"));
        cap->set(CV_CAP_PROP_FRAME_HEIGHT, pt.get<int>("camera.height"));
        cap->set(CV_CAP_PROP_FPS, desired_fps);
        cap->set(CV_CAP_PROP_CONVERT_RGB, pt.get<bool>("camera.convert_rgb"));
    }

    input = new Mat();
    if(test_image) {
        Mat image = imread(video_file);
        image.copyTo(*input);
    } else {
        cap->read(*input);
    }

    /*
     * If the save_video config entry is something other than "none", create
     * a video encoding instance using the MJPG format.
     * This uses ffmpeg under the hood and technically supports any FOURCC codes
     * that ffmpeg does, but MJPG has a good performance/size tradeoff and works
     * very well on the Pi, in lieu of being able to easily use the hardware
     * h.264 encoder.
     */
    int out_scale_w = scale_w, out_scale_h = scale_h;
    if(out_scale_w <= 0 || out_scale_h <= 0) {
        out_scale_w = input->cols;
        out_scale_h = input->rows;
    }
    wri = new VideoWriter();
    if(save_video_file.compare("none") != 0) {
        save_video = true;
        int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
        wri->open(save_video_file, fourcc,
                  // set output FPS to default if input FPS == 0
                  (desired_fps <= 0) ? default_fps : desired_fps,
                  Size(out_scale_w, out_scale_h)
        );
    }

    output = Mat(input->rows, input->cols, CV_8UC3);
    img_center = Point(input->cols / 2, input->rows / 2);

    /*
     * Create our marker detection, target finder, target, camera model and
     * navigator instances...
     */
    tf = new TargetFinder();
    target = new PersistentTarget();
    cm = new CameraModel(
        pt.get<double>("target.width"),
        pt.get<double>("target.height"),
        pt.get<double>("camera.focal_length"),
        pt.get<double>("camera.magnification_factor"),
        pt.get<double>("camera.sensor_width")
    );
    nv = new Navigator(input->cols, input->rows);

    /*
     * ...and pass them a bunch of configuration stuff from the .ini files.
     */
    if(pt.get<string>("parameters.method").compare("cascade") == 0) {
        detector = new CascadeDetector();
        #define cas ((CascadeDetector *)detector)
        cas->setClassifier(pt.get<string>("parameters.cascade.classifier_file"));
        cas->setMinSize(pt.get<int>("parameters.cascade.min_size"));
        cas->setMinNeighbors(pt.get<int>("parameters.cascade.min_neighbors"));
        cas->setScaleFactor(pt.get<double>("parameters.cascade.scale_factor"));
    } else {
        detector = new FSMDetector();
        #define fsm ((FSMDetector *) detector)
        fsm->setRowStep(pt.get<int>("parameters.fsm.row_step"));
        fsm->setMinThreshold(pt.get<int>("parameters.fsm.min_threshold"));
        fsm->setMinLength(pt.get<int>("parameters.fsm.min_length"));
        fsm->setSmoothness(pt.get<double>("parameters.fsm.threshold_smoothness"));
        fsm->setTolerance(pt.get<double>("parameters.fsm.tolerance"));
        fsm->setMarkerAspectTolerance(pt.get<double>("parameters.fsm.marker_aspect_tolerance"));
        fsm->setHomogeneity(pt.get<bool>("parameters.fsm.check_homogeneity"));
        fsm->setOtsu(pt.get<bool>("parameters.fsm.thresh_otsu"));
    }
    name_str_formatted = format("method = %s", detector->getName().c_str());
    output_fsm_states = output_fsm_states && (0 == detector->getName().compare("fsm"));
    nv->setPIDs(
            pt.get<double>("navigator.pitch_p"),
            pt.get<double>("navigator.pitch_i"),
            pt.get<double>("navigator.pitch_d"),
            pt.get<double>("navigator.roll_p"),
            pt.get<double>("navigator.roll_i"),
            pt.get<double>("navigator.roll_d")
    );
    target->setTimeout(getTickFrequency() * pt.get<double>("target.lifetime"));
    target->setAngleOffset(pt.get<double>("camera.angle_offset"));
    tf->setMarkerDistances(
            pt.get<double>("target.min_marker_distance"),
            pt.get<double>("target.max_marker_distance")
    );
    tf->setMarkerSizeTolerance(pt.get<double>("target.marker_size_tolerance"));

    /*
     * Some more miscellaneous config stuff loaded into local variables
     */
    target_influence = pt.get<double>("target.alpha");
    min_age = pt.get<int>("target.min_age");
    convert_yuv = pt.get<bool>("camera.convert_yuv");
    flip_vertical = pt.get<bool>("camera.flip_vertical");
    flip_horizontal = pt.get<bool>("camera.flip_horizontal");
    wait_for_input = pt.get<bool>("gui.wait_for_input");
}


void tick() {
    if(wait_for_input) {
        getchar();
    }
    t_start = getTickCount();
    if (!test_image) {
        if(!cap->read(*input)) {
            DEBUGPRINT("couldn't get video frame");
            waitKey(1000);
            if(!endless_video) {
                running = false;
                return;
            }
        }
    }
    t_framegrab = getTickCount();

    if(output_marker_info && !test_image) {
        marker_info = ((ImageCycler*)cap)->getFile();
    }

    /*
     * Flip the image vertically and horizontally if needed
     */
    if(flip_vertical && flip_horizontal) {
        flip(*input, *input, -1);
    } else if(flip_vertical && !flip_horizontal) {
        flip(*input, *input, 0);
    } else if(flip_horizontal && !flip_vertical) {
        flip(*input, *input, 1);
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
        cvtColor(*input, *input, COLOR_BGR2YUV);
    }

    t_preprocessing = getTickCount();

    /*
     * Run the algorithm for finding potential target markers in the input
     * image.
     */
    vector<shared_ptr<Marker>> markers = detector->detect(
            *input, output, (render_detector && (save_video | !headless))
    );
    if(output_fsm_states) {
        std::cout << ((FSMDetector *)detector)->strStateCounts() << std::endl;
    }
    t_detect = getTickCount();

    /*
     * This returns a list of potential targets -- regardless of
     * whether they're valid or not.
     * Valid ones are .calc_valid == true.
     * (calc_valid is set when running the Target::valid() function, but
     * this is called inside the target recognition algorithm already)
     */
    vector<Target> targets = tf->groupTargets(
            *input, output, markers,
            render_markers, marker_info
    );
    t_group = getTickCount();

    /*
     * Now we (possibly) have a list of targets, we need to select the best
     * one.
     * To do this, we calculate the euclidean distance between each target
     * and the existing "persistent target". The one with the smallest
     * distance is chosen as the best candidate target. Only valid targets
     * are considered.
     */
    Target *best_target = nullptr;
    double best_similarity = -1.0;
    for(int i=0; i<targets.size(); i++) {
        if(!targets[i].valid()) {
            continue;
        }
        double cur_similarity = target->similarity(&targets[i]);
        if(best_similarity == -1.0 || cur_similarity < best_similarity) {
            best_similarity = cur_similarity;
            best_target = &targets[i];
        }
    }
    if(best_target) {
        best_target->calcGeometry();
    }
    t_select = getTickCount();

    /*
     * Show which marker is the corner of the target, for debug purposes
     */
    if(render_target) {
        if(best_target) {
            shared_ptr<Marker> *this_markers = best_target->getMarkers();
            for(int i=0; i<3; i++) {
                Point this_center = this_markers[i]->center();
                const char *markerStr = "%d";
                putText(output, format(markerStr, i), this_center,
                            FONT_HERSHEY_SIMPLEX, 0.5, c_black, 4);
                putText(output, format(markerStr, i), this_center,
                            FONT_HERSHEY_SIMPLEX, 0.5, c_yellow, 2);
            }

            shared_ptr<Marker> best_corner = best_target->getCorner();
            Rect corner_rect = best_corner->rect();
            rectangle(output, corner_rect, c_yellow, 2);
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
        cout << "{\"target\": ";
    }

    /*
     * If the target is "alive" (0 <= target age < maximum lifetime), we
     * send its information to the Navigator instance, as well as output the
     * target info. in JSON format.
     */
    if(target->alive(t_start) && target->age() >= min_age) {
        Rect r = target->rect();
        Point center = target->center();
        double velocity = target->velocity();
        double real_distance = cm->distance(
            input->cols, input->rows,
            r.width, r.height) * 0.001;
        double angle = target->angle();
        if(best_similarity != 0.0) {
            nv->update(center, angle, real_distance, target->age(), velocity);
        } else {
            nv->update(center, angle, real_distance, target->age(), velocity);
        }
        if(output_target) {
            cout << "{" << target->str();
            cout << ", \"distance(m)\": " << real_distance << "}";
        }
        if ((!headless || save_video) && render_target) {
            rectangle(output, r, c_black, 3);
            rectangle(output, r, c_red, 1);
            circle(output, center, 5, c_black, -1);
            Point endPoint = Point(
                    center.x + r.width * cos(angle),
                    center.y + r.height * sin(angle)
            );
            line(output, center, endPoint, c_black, 3);
            line(output, center, endPoint, c_red, 1);
            circle(output, center, 3, c_red, -1);
            const char *distanceStr = "%0.2f m";
            putText(output, format(distanceStr, real_distance), center,
                        FONT_HERSHEY_SIMPLEX, 0.7, c_black, 4);
            putText(output, format(distanceStr, real_distance), center,
                        FONT_HERSHEY_SIMPLEX, 0.7, c_green, 2);
        }
    } else {
        /*
         * If the target is dead, we ask the Navigator to return to center.
         */
        if(output_target) {
            cout << "null";
        }
        nv->update(fps_delta);
    }

    if(output_target && output_navigator) {
        cout << ", \"sticks\": {" << nv->str() << "}";
    } if(output_target) {
        cout << ", \"fps\": " << fps << ", \"time\": " << t_start <<
        "}" << endl;
    }

    /*
     * If we are writing a video or outputting to screen, render some
     * more stuff to the output image -- FPS in top left corner,
     * navigator represented as a crosshair with dots showing the pitch/roll
     * values.
     */
    if(!headless || save_video) {
        if(render_fps) {
            string tickStrFormatted = format("fps = %0.0f", fps);
            Point fpsPos(10, 20);
            Point namePos(10, 36);
            putText(output, tickStrFormatted, fpsPos,
                        FONT_HERSHEY_SIMPLEX, 0.5, c_black,
                        3);
            putText(output, tickStrFormatted, fpsPos,
                        FONT_HERSHEY_SIMPLEX, 0.5,
                        c_yellow);
            putText(output, name_str_formatted, namePos,
                        FONT_HERSHEY_SIMPLEX, 0.5, c_black, 3);
            putText(output, name_str_formatted, namePos,
                        FONT_HERSHEY_SIMPLEX, 0.5, c_yellow);
        }
        if(render_navigator) {
            int ll = 150;
            line(output, Point(img_center.x, img_center.y - ll),
                     Point(img_center.x, img_center.y + ll),
                     c_black, 3);
            line(output, Point(img_center.x - ll, img_center.y),
                     Point(img_center.x + ll, img_center.y),
                     c_black, 3);
            line(output, Point(img_center.x, img_center.y - ll),
                     Point(img_center.x, img_center.y + ll),
                     c_white, 1);
            line(output, Point(img_center.x - ll, img_center.y),
                     Point(img_center.x + ll, img_center.y),
                     c_white, 1);
            circle(output, nv->image_point(0, ll), 5,
                       c_black, -1);
            circle(output, nv->image_point(0, ll), 3,
                       c_green, -1);
            circle(output, nv->image_point(1, ll), 5,
                       c_black, -1);
            circle(output, nv->image_point(1, ll), 3,
                       c_green, -1);
        }
    }
    if((!headless || save_video) && (scale_w > 0 && scale_h > 0)) {
        resize(output, output, Size(scale_w, scale_h));
    }
    t_render = getTickCount();

    if(save_video) {
        wri->write(output);
    }
    if(save_snapshots && target->alive(t_start) && target->age() <= 1) {
        imwrite(format(save_snapshot_path.c_str(), target_count++), output);
    }
    if(!headless) {
        imshow("output", output);
        waitKey(wait_key);
    }
    t_output = getTickCount();
    double tickFreq = getTickFrequency();

    // Output times spent on each part of the loop.
    if(output_times) {
        cout << "framegrab=" << ((t_framegrab - t_start) / tickFreq);
        cout << ", preprocess=" << ((t_preprocessing - t_framegrab) / tickFreq);
        cout << ", detect=" << ((t_detect - t_preprocessing) / tickFreq);
        cout << ", group=" << ((t_group - t_detect) / tickFreq);
        cout << ", select=" << ((t_select - t_group) / tickFreq);
        cout << ", render=" << ((t_render - t_select) / tickFreq);
        cout << ", output=" << ((t_output - t_render) / tickFreq);
    }

    // Calculate overall framerate.
    t_end = getTickCount();
    fps_delta = (t_end - t_start) / tickFreq;
    fps_hz = 1.0 / fps_delta;
    fps += alphafps * (fps_hz - fps);
    if(output_times) {
        cout << ", fps=" << fps;
        cout << endl;
    }

}


int main(int argc, char* argv[]) {
    init(argc, argv);
    running = true;

    if(pi_build && async_mode) {
        /*
         * Set PiCam to use our tick() function as the callback function.
         */
        DEBUGPRINT("Pi Camera MMAL stuff not implemented yet. Exiting");
        return 1;
    } else {
        /*
        * Main loop
        */
        while (running) {
            tick();
        }
    }

    return 0;
}

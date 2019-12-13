#include <pangolin_viewer/viewer.h>

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/camera/perspective.h>

#include "zed_camera.hpp"
#include "realsense_camera.hpp"
#include "mynteye_camera.hpp"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <tuple>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <popl.hpp>

#include <spdlog/spdlog.h>

#ifdef USE_BACKWARD

#ifndef __GNUC__
#error "USE_BACKWARD only supports gcc"
#endif // __GNUC__

#include <backward.hpp>

namespace backward {
    backward::SignalHandling sh;

    using Rethrower = void (*)(void*, void*, void(*)(void*));
    backward::StackTrace st;
    backward::Printer printer;
    std::atomic<bool> has_new_trace {false};

    Rethrower rethrow __attribute__ ((noreturn)) = reinterpret_cast<Rethrower>(dlsym(RTLD_NEXT, "__cxa_throw"));

    void print(void)
    {
        if (has_new_trace) {
            printer.print(st);
            has_new_trace = false;
        }
    }
} // namespace backward

// Nasty trick to capture the stack trace of an exception (with gcc)
extern "C" {
    void __cxa_throw(void *ex, void *info, void (*dest)(void *)) {
        backward::st.load_here(32);
        backward::st.skip_n_firsts(1);
        backward::has_new_trace = true;
        backward::rethrow(ex, info, dest);
    }
}
#endif // USE_BACKWARD

namespace {

template <typename F, typename... Ts>
double timeit(F&& f, Ts&&...args)
{
    const auto tp_1 = std::chrono::steady_clock::now();
    std::forward<F>(f)(std::forward<Ts>(args)...);
    const auto tp_2 = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
}

[[noreturn]] void custom_rethrower(void)
{
#ifdef USE_BACKWARD
        backward::print();
#endif
        std::rethrow_exception(std::current_exception());
}

} // anonymous namespace

template <typename CameraType>
class StereoSLAM {

public:
    explicit StereoSLAM(
            const std::string& config_file_path,
            const std::string& input_video_file_path,
            const std::string& output_video_file_path,
            const std::string& vocab_file_path,
            const std::string& input_map_db_path,
            const std::string& output_map_db_path,
            const unsigned int frame_skip,
            const bool no_sleep,
            const bool auto_term,
            const bool eval_log,
            const bool localization
            )
        : _camera {config_file_path, input_video_file_path, output_video_file_path}
        , _cfg {std::make_shared<openvslam::config>(_camera.get_config(), "")}
        , _vocab_file_path {vocab_file_path}
        , _input_map_db_path {input_map_db_path}
        , _output_map_db_path {output_map_db_path}
        , _frame_skip {frame_skip}
        , _no_sleep {no_sleep}
        , _auto_term {auto_term}
        , _eval_log {eval_log}
        , _localization {localization}
        , _track_times {}
        , _slam(_cfg, _vocab_file_path)
        , _viewer(_cfg, &_slam, _slam.get_frame_publisher(), _slam.get_map_publisher())
    {
        spdlog::info("Opened {0} camera {1}x{2} @ {3} FPS", _cfg->camera_->name_, _cfg->camera_->cols_, _cfg->camera_->rows_, _cfg->camera_->fps_);
        spdlog::info("Running in {0} mode", _cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo ? "stereo" : "mono");

        if (!_input_map_db_path.empty()) {
            // load the prebuilt map
            _slam.load_map_database(_input_map_db_path);
            spdlog::info("Loaded map from {0}", _input_map_db_path);
        }

        // Initialise a new map if it was not loaded from file
        _slam.startup(_input_map_db_path.empty());
        spdlog::info("Started up SLAM system");

        if (_localization) {
            _slam.disable_mapping_module();
            spdlog::info("Mapping module disabled");
        }

        // run the SLAM in another thread
        std::thread thread([&]() {
            try {
                _slam_loop();
            }
            catch (...) {
                _crash_handler();
            }
        });

        // run the viewer in the current thread
        try {
            _viewer.run();
        }
        catch (...) {
            _crash_handler();
        }

        thread.join();
    }

    virtual ~StereoSLAM()
    {
        // shutdown the SLAM process
        _slam.shutdown();

        if (!_localization && _eval_log) {
            // output the trajectories for evaluation
            _slam.save_frame_trajectory("frame_trajectory.txt", "TUM");
            _slam.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
            // output the tracking times for evaluation
            std::ofstream ofs("track_times.txt", std::ios::out);
            if (ofs.is_open()) {
                for (const auto track_time : _track_times) {
                    ofs << track_time << std::endl;
                }
                ofs.close();
            }
        }

        if (!_output_map_db_path.empty()) {
            // output the map database
            _slam.save_map_database(_output_map_db_path);
        }

        std::sort(_track_times.begin(), _track_times.end());
        const auto total_track_time = std::accumulate(_track_times.begin(), _track_times.end(), 0.0);
        std::cout << "median tracking time: " << _track_times.at(_track_times.size() / 2) << "[s]" << std::endl;
        std::cout << "mean tracking time: " << total_track_time / _track_times.size() << "[s]" << std::endl;
    }

private:

    void _slam_loop(void)
    {
        unsigned int num_frame = 0;
        double track_time = 0.0;
        cv::Mat frame_left, frame_right;
        double timestamp = 0.0;

        while (_camera.has_frames()) {
            if (_slam.tracker_is_paused()) {
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(1e6 / _cfg->camera_->fps_)));
                continue;
            }

            if (!_camera.grab(frame_left, frame_right, timestamp)) {
                spdlog::warn("Failed to grab frame at time {0}", timestamp);
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(1e6 / _cfg->camera_->fps_)));
                continue;
            }

            // input the current frame and estimate the camera pose
            if (num_frame % _frame_skip == 0) {
                track_time = timeit([&]() {_slam.feed_stereo_frame(frame_left, frame_right, timestamp);});
            }

            if (num_frame % _frame_skip == 0) {
                _track_times.push_back(track_time);
            }

            // wait until the timestamp of the next frame
            if (!_no_sleep) {
                const auto wait_time = 1.0 / _cfg->camera_->fps_ - track_time;
                if (0.0 < wait_time) {
                    std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(wait_time * 1e6)));
                }
            }

            ++num_frame;

            // check if the termination of SLAM system is requested or not
            if (_slam.terminate_is_requested()) {
                break;
            }
        }

        // wait until the loop BA is finished
        while (_slam.loop_BA_is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }

        // automatically close the viewer
        if (_auto_term) {
            _viewer.request_terminate();
        }
    }

    void _crash_handler(void)
    {
        _camera.cleanup();
        custom_rethrower();
    }

    CameraType _camera;
    std::shared_ptr<openvslam::config> _cfg;
    const std::string _vocab_file_path;
    const std::string _input_map_db_path;
    const std::string _output_map_db_path;
    const unsigned int _frame_skip;
    const bool _no_sleep;
    const bool _auto_term;
    const bool _eval_log;
    const bool _localization;
    std::vector<double> _track_times;
    openvslam::system _slam;
    pangolin_viewer::viewer _viewer;
};

int main(int argc, char* argv[])
{
    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto camera_model = op.add<popl::Value<std::string>>("m", "camera-model", "camera model (ZED, MYNT, RealSense)");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto input_video_file_path = op.add<popl::Value<std::string>>("", "input-video", "input video file path", "");
    auto output_video_file_path = op.add<popl::Value<std::string>>("", "output-video", "output video file path", "");
    auto frame_skip = op.add<popl::Value<unsigned int>>("", "frame-skip", "interval of frame skip", 1);
    auto no_sleep = op.add<popl::Switch>("", "no-sleep", "not wait for next frame in real time");
    auto auto_term = op.add<popl::Switch>("", "auto-term", "automatically terminate the viewer");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto input_map_db_path = op.add<popl::Value<std::string>>("", "input-map-db", "load a map database at this path before SLAM", "");
    auto output_map_db_path = op.add<popl::Value<std::string>>("", "output-map-db", "store a map database at this path after SLAM", "");
    auto localization = op.add<popl::Switch>("", "localization", "run in localization mode");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !config_file_path->is_set() || !camera_model->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!localization->is_set() && !output_map_db_path->is_set()) {
        std::cerr << "No map output file provided as argument. "
                  << "A map destination file is required "
                  << "to run in mapping mode."
                  << std::endl << std::endl
                  << op
                  << std::endl;
        return EXIT_FAILURE;
    }
    if (localization->is_set() && !input_map_db_path->is_set()) {
        std::cerr << "No map provided as argument. A map is required "
                  << "to run in localization mode."
                  << std::endl << std::endl
                  << op
                  << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }


    auto const args = std::make_tuple(
            config_file_path->value(),
            input_video_file_path->value(),
            output_video_file_path->value(),
            vocab_file_path->value(),
            input_map_db_path->value(),
            output_map_db_path->value(),
            frame_skip->value(),
            no_sleep->is_set(),
            auto_term->is_set(),
            eval_log->is_set(),
            localization->value()
            );

    // Run SLAM
    try {
        if (camera_model->value() == "ZED") {
            std::make_from_tuple<StereoSLAM<ZedCamera>>(args);
        }
        else if (camera_model->value() == "MYNT") {
            std::make_from_tuple<StereoSLAM<MyntEyeCamera>>(args);
        }
        else if (camera_model->value() == "RealSense") {
            std::make_from_tuple<StereoSLAM<RealSenseCamera>>(args);
        }
        else {
            throw std::runtime_error("Unknown camera model \"" + camera_model->value() + "\"");
        }
    }
    catch (...) {
        custom_rethrower();
    }

    return EXIT_SUCCESS;
}

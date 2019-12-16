#pragma once

#include "stack_trace.hpp"

#include "zed_camera.hpp"
#include "realsense_camera.hpp"
#include "mynteye_camera.hpp"

#include <pangolin_viewer/viewer.h>

#include <openvslam/camera/perspective.h>
#include <openvslam/config.h>
#include <openvslam/system.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <spdlog/spdlog.h>

namespace {

template <typename F, typename... Ts>
double timeit(F&& f, Ts&&...args)
{
    const auto tp_1 = std::chrono::steady_clock::now();
    std::forward<F>(f)(std::forward<Ts>(args)...);
    const auto tp_2 = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
}

} // anonymous namespace

namespace CameraSlam {

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
        const bool rgbd_mode = _cfg->yaml_node_["Camera.setup"].as<std::string>() == "RGBD";
        unsigned int num_frame = 0;
        double track_time = 0.0;
        cv::Mat frame_left, frame_right, rgb, depth;
        double timestamp = 0.0;

        while (_camera.has_frames()) {
            if (_slam.tracker_is_paused()) {
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(1e6 / _cfg->camera_->fps_)));
                continue;
            }

            if (!_camera.grab(frame_left, frame_right, rgb, depth, timestamp)) {
                spdlog::warn("Failed to grab frame at time {0}", timestamp);
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<unsigned int>(1e6 / _cfg->camera_->fps_)));
                continue;
            }

            // input the current frame and estimate the camera pose
            if (num_frame % _frame_skip == 0) {
                if (rgbd_mode) {
                    track_time = timeit([&]() {_slam.feed_RGBD_frame(rgb, depth, timestamp);});
                }
                else {
                    track_time = timeit([&]() {_slam.feed_stereo_frame(frame_left, frame_right, timestamp);});
                }
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

} // namespace CameraSlam

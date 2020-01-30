#include "stereo_slam.hpp"
#include "stack_trace.hpp"

#include "realsense_camera.hpp"
#include "mynteye_camera.hpp"

#if CAMERA_SLAM_WITH_ZED
    #include "zed_camera.hpp"
#endif // CAMERA_SLAM_WITH_ZED

#include <iostream>

#include <popl.hpp>
#include <spdlog/spdlog.h>

int main(int argc, char* argv[])
{
#if CAMERA_SLAM_WITH_ZED
    std::string camera_model_help = "camera model (ZED, MYNT, RealSense)";
#else
    std::string camera_model_help = "camera model (MYNT, RealSense)";
#endif // CAMERA_SLAM_WITH_ZED

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto camera_model = op.add<popl::Value<std::string>>("m", "camera-model", camera_model_help);
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
#if CAMERA_SLAM_WITH_ZED
            std::make_from_tuple<CameraSlam::StereoSLAM<CameraSlam::ZedCamera>>(args);
#else
            throw std::runtime_error("This build does not support the ZED camera");
#endif // CAMERA_SLAM_WITH_ZED
        }
        else if (camera_model->value() == "MYNT") {
            std::make_from_tuple<CameraSlam::StereoSLAM<CameraSlam::MyntEyeCamera>>(args);
        }
        else if (camera_model->value() == "RealSense") {
            std::make_from_tuple<CameraSlam::StereoSLAM<CameraSlam::RealSenseCamera>>(args);
        }
        else {
            throw std::runtime_error("Unknown camera model \"" + camera_model->value() + "\"");
        }
    }
    catch (...) {
        CameraSlam::custom_rethrower();
    }

    return EXIT_SUCCESS;
}

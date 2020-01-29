#pragma once

#include "stereo_camera.hpp"

// Dependencies are not lint-clean
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    #pragma GCC diagnostic ignored "-Wreorder"
    #pragma GCC diagnostic ignored "-Wignored-qualifiers"
    #pragma GCC diagnostic ignored "-Wpedantic"

    #if __GNUC__ >= 9
        #pragma GCC diagnostic ignored "-Wdeprecated-copy"
    #endif
#endif // defined(__GNUC__) || defined(__clang__)

#include <sl/Camera.hpp>

#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic pop
#endif // defined(__GNUC__) || defined(__clang__)

namespace CameraSlam {

class ZedCamera : public StereoCamera {

public:
    explicit ZedCamera(
            const std::string& config_file_path,
            const std::string& input_svo_file_path,
            const std::string& output_svo_file_path
            );

    virtual void cleanup(void) override;
    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& rgb, cv::Mat& depth, double& timestamp) override;
    virtual bool has_frames(void) override;

private:
    double _timestamp(void);
    sl::InitParameters _make_zed_parameters(void);
    void _read_zed_camera_calibration(void);

    sl::Camera _zed;
    sl::Mat _frame_left;
    sl::Mat _frame_right;
    sl::Mat _rgb;
    sl::Mat _depth;
};

} // namespace CameraSlam

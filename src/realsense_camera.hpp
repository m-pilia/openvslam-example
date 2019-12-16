#pragma once

#include "stereo_camera.hpp"

#include <librealsense2/rs.hpp>

namespace CameraSlam {

class RealSenseCamera : public StereoCamera
{
public:
    explicit RealSenseCamera(
            const std::string& config_file_path,
            const std::string& input_file_path,
            const std::string& output_file_path
            );

    virtual void cleanup(void) override;
    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp) override;
    virtual bool has_frames(void) override;

private:
    enum StreamIndex {
        Left = 1,
        Right = 2,
    };

    rs2::config _make_realsense_config(void);
    void _read_camera_calibration(void);

    rs2::pipeline _pipe;
    rs2::video_frame _frame_left;
    rs2::video_frame _frame_right;
};

} // namespace CameraSlam

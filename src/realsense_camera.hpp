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
    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& rgb, cv::Mat& depth, double& timestamp) override;
    virtual bool has_frames(void) override;

private:
    enum StreamIndex {
        Depth = 0,
        Left = 1,
        Right = 2,
    };

    constexpr static struct {
        const int rows;
        const int cols;
        const float fps;
        const rs2_format format;
    } T265 = {800, 848, 30.0f, RS2_FORMAT_Y8};

    rs2::config _make_realsense_config(void);
    void _read_camera_calibration(void);
    rs2_stream _get_stream_type(void);
    rs2_intrinsics _get_intrinsics(const StreamIndex stream_index);
    rs2_extrinsics _get_extrinsics(void);

    rs2::context _context;
    rs2::pipeline _pipe;
    rs2::video_frame _frame_left;
    rs2::video_frame _frame_right;
    rs2::video_frame _rgb;
    rs2::video_frame _depth;
    float _depth_scale;
};

} // namespace CameraSlam

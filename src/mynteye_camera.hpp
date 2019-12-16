#pragma once

#include "stereo_camera.hpp"

#include <fstream>
#include <queue>
#include <tuple>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <mynteye/api/api.h>

// Dependencies are not lint-clean
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif // defined(__GNUC__) || defined(__clang__)

#include <spdlog/spdlog.h>

#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic pop
#endif // defined(__GNUC__) || defined(__clang__)

namespace CameraSlam {

class MyntEyeCamera : public StereoCamera
{
public:
    explicit MyntEyeCamera(
            const std::string& config_file_path,
            const std::string& input_file_path,
            const std::string& output_file_path
            );

    virtual ~MyntEyeCamera();
    virtual void cleanup(void) override;
    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp) override;
    virtual bool has_frames(void) override;

private:
    enum DataType {LEFT, RIGHT, IMU};

    struct TimedImage {
        TimedImage(const cv::Mat& img, const uint64_t ts);
        const cv::Mat image;
        const uint64_t timestamp;
    };

    std::shared_ptr<mynteye::API> _api;
    mynteye::api::StreamData _left_data, _right_data;
    std::unique_ptr<std::ifstream> _input_file;
    std::unique_ptr<std::ofstream> _output_file;
    bool _has_frames;
    std::mutex _mtx;
    std::queue<TimedImage> _queue_left, _queue_right;
    const mynteye::StreamRequest _stream_request {
            752,                   // width
            480,                   // height
            mynteye::Format::YUYV, // pixel format
            60,                    // fps
    };
    static constexpr size_t _queue_max_length = 10;

    void _read_header(void);
    void _write_header(void);
    void _read_camera_calibration(void);
    void _enqueue(const cv::Mat& image, const uint64_t timestamp, const DataType data_type);
    void _dequeue_image_pair(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp);
    void _write_image(const mynteye::api::StreamData& data, const DataType data_type);
    bool _read_datum(cv::Mat& image, uint64_t& timestamp, DataType& data_type);
    bool _read_image_pair(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp);
    bool _grab_image_pair(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp);
    void _discard_old_image(void);
};

} // namespace CameraSlam

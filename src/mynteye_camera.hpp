#pragma once

#include "stereo_camera.hpp"

#include <opencv2/core/core.hpp>
#include <mynteye/api/api.h>

class MyntEyeCamera : public StereoCamera
{
public:
    explicit MyntEyeCamera(
            const std::string& config_file_path,
            const std::string& input_file_path,
            const std::string& output_file_path
            )
        : StereoCamera(config_file_path, input_file_path, output_file_path)
        , _api {mynteye::API::Create(0, {})}
    {
        if (!_api) {
            throw std::runtime_error("Failed to create MyntEye API");
        }

        _api->ConfigStreamRequest(_stream_request);
        _api->EnableStreamData(mynteye::Stream::LEFT_RECTIFIED);
        _api->EnableStreamData(mynteye::Stream::RIGHT_RECTIFIED);
        _api->Start(mynteye::Source::ALL);

        _yaml_node["Camera.name"] = "MYNT EYE S1030-IR";
        _yaml_node["Camera.setup"] = "stereo";
        _yaml_node["Camera.model"] = "perspective";

        _read_camera_calibration();
    }

    virtual ~MyntEyeCamera()
    {
        cleanup();
    }

    virtual void cleanup(void)
    {
        _api->Stop(mynteye::Source::ALL);
    }

    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp)
    {
        _api->WaitForStreams();

        auto &&left_data = _api->GetStreamData(mynteye::Stream::LEFT_RECTIFIED);
        auto &&right_data = _api->GetStreamData(mynteye::Stream::RIGHT_RECTIFIED);
        if (left_data.frame.empty() || right_data.frame.empty()) {
            return false;
        }

        timestamp = static_cast<double>(left_data.img->timestamp) / 1e6;
        frame_left = left_data.frame; // FIXME convert to RGB?
        frame_right = right_data.frame;
        return true;
    }

    virtual bool end_of_frames(void)
    {
        return false; // FIXME
    }

private:
    std::shared_ptr<mynteye::API> _api;
    const mynteye::StreamRequest _stream_request {
            752,                   // width
            480,                   // height
            mynteye::Format::YUYV, // pixel format
            60,                    // fps
    };

    void _read_camera_calibration(void)
    {
        // intrinsics = ?
        // auto const extrinsics = _api->GetExtrinsics(mynteye::Stream::RIGHT_RECTIFIED, mynteye::Stream::LEFT_RECTIFIED);

        _yaml_node["Camera.rows"] = _stream_request.height;
        _yaml_node["Camera.cols"] = _stream_request.width;
        _yaml_node["Camera.fps"] = _stream_request.fps;
        _yaml_node["Camera.color_order"] = "RGB"; // FIXME not RGB
        // _yaml_node["Camera.fx"] = intrinsics.fx;
        // _yaml_node["Camera.fy"] = intrinsics.fy;
        // _yaml_node["Camera.cx"] = intrinsics.cx;
        // _yaml_node["Camera.cy"] = intrinsics.cy;
        // _yaml_node["Camera.k1"] = 0.0;
        // _yaml_node["Camera.k2"] = 0.0;
        // _yaml_node["Camera.p1"] = 0.0;
        // _yaml_node["Camera.p2"] = 0.0;
        // _yaml_node["Camera.k3"] = 0.0;
        // _yaml_node["Camera.focal_x_baseline"] = extrinsics.translation[0] * intrinsics.fx;
        // _yaml_node["Camera.color_order"] = "Gray";
    }
};

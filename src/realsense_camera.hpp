#include "stereo_camera.hpp"

#include <librealsense2/rs.hpp>

namespace {

// OBS! The `frame` object retains ownership of the image buffer.
cv::Mat frame_to_cv(rs2::video_frame frame)
{
    return cv::Mat(cv::Size(frame.get_width(), frame.get_height()), CV_8UC1, const_cast<void*>(frame.get_data()));
}

} // anonymous namespace

class RealSenseCamera : public StereoCamera
{
public:
    explicit RealSenseCamera(
            const std::string& config_file_path,
            const std::string& input_file_path,
            const std::string& output_file_path
            )
        : StereoCamera(config_file_path, input_file_path, output_file_path)
        , _pipe {}
        , _frame_left {rs2::frame()}
        , _frame_right {rs2::frame()}
    {
        if (_playback && _recording) {
            throw std::runtime_error("Cannot record and play at the same time");
        }

        _yaml_node["Camera.name"] = "RealSense D435";
        _yaml_node["Camera.setup"] = "stereo";
        _yaml_node["Camera.model"] = "perspective";
        _yaml_node["Camera.color_order"] = "Gray";

        // Start pipeline
        rs2::pipeline_profile pipe_profile = _pipe.start(_make_realsense_config());

        // Turn off IR projector
        if (!_playback) {
            for (auto const& element : pipe_profile.get_device().query_sensors()) {
                if (strcmp(element.get_info(RS2_CAMERA_INFO_NAME), "Stereo Module") == 0) {
                    element.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 0u);
                }
            }
        }

        _read_camera_calibration();
    }

    virtual void cleanup(void) override
    {
        _pipe.stop();
    }

    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp) override
    {
        rs2::frameset frameset = _pipe.wait_for_frames();
        _frame_left = frameset.get_infrared_frame(StreamIndex::Left);
        _frame_right = frameset.get_infrared_frame(StreamIndex::Right);

        if (!_frame_left || !_frame_right) {
            return false;
        }

        timestamp = static_cast<double>(_frame_left.get_timestamp()) / 1e3;
        frame_left = frame_to_cv(_frame_left);
        frame_right = frame_to_cv(_frame_right);
        return true;
    }

    virtual bool end_of_frames(void) override
    {
        if (_playback) {
            auto const& device = _pipe.get_active_profile().get_device().as<rs2::playback>();
            auto position = device.get_position();
            auto duration = static_cast<uint64_t>(device.get_duration().count());
            return position >= duration;
        }
        return false;
    }

private:
    enum StreamIndex {
        Left = 1,
        Right = 2,
    };

    rs2::config _make_realsense_config(void)
    {
        // Set camera settings
        auto const rows = _yaml_node["Camera.rows"].as<int>();
        auto const cols = _yaml_node["Camera.cols"].as<int>();
        auto const fps = static_cast<int>(std::round(_yaml_node["Camera.fps"].as<float>()));

        rs2::config config;

        if (_playback) {
            config.enable_device_from_file(_input_file_path);
        }
        else {
            config.enable_stream(RS2_STREAM_INFRARED, StreamIndex::Left, cols, rows, RS2_FORMAT_Y8, fps);
            config.enable_stream(RS2_STREAM_INFRARED, StreamIndex::Right, cols, rows, RS2_FORMAT_Y8, fps);
        }

        if (_recording) {
            config.enable_record_to_file(_output_file_path);
        }

        return config;
    }

    void _read_camera_calibration(void)
    {
        auto const left_stream = _pipe.get_active_profile().get_stream(RS2_STREAM_INFRARED, StreamIndex::Left).as<rs2::video_stream_profile>();
        auto const right_stream = _pipe.get_active_profile().get_stream(RS2_STREAM_INFRARED, StreamIndex::Right).as<rs2::video_stream_profile>();
        auto const intrinsics = left_stream.get_intrinsics();

        _yaml_node["Camera.fx"] = intrinsics.fx;
        _yaml_node["Camera.fy"] = intrinsics.fy;
        _yaml_node["Camera.cx"] = intrinsics.ppx;
        _yaml_node["Camera.cy"] = intrinsics.ppy;
        _yaml_node["Camera.k1"] = intrinsics.coeffs[0];
        _yaml_node["Camera.k2"] = intrinsics.coeffs[1];
        _yaml_node["Camera.p1"] = intrinsics.coeffs[2];
        _yaml_node["Camera.p2"] = intrinsics.coeffs[3];
        _yaml_node["Camera.k3"] = intrinsics.coeffs[4];
        _yaml_node["Camera.focal_x_baseline"] = right_stream.get_extrinsics_to(left_stream).translation[0] * intrinsics.fx;
        _yaml_node["Camera.color_order"] = "Gray";
    }

    rs2::pipeline _pipe;
    rs2::video_frame _frame_left;
    rs2::video_frame _frame_right;
};

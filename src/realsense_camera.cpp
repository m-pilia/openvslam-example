#include "realsense_camera.hpp"

namespace {

// OBS! The `frame` object retains ownership of the image buffer.
cv::Mat frame_to_cv(rs2::video_frame frame)
{
    int cv_type = -1;
    switch (frame.get_profile().format()) {

    // 16-bit linear depth values. The depth is meters is equal to depth
    // scale * pixel value.
    case RS2_FORMAT_Z16:
        cv_type = CV_16UC1;
        break;
    // 16-bit float-point disparity values. Depth->Disparity conversion
    // : Disparity = Baseline*FocalLength/Depth.
    case RS2_FORMAT_DISPARITY16:
        break;
    // 32-bit floating point 3D coordinates.
    case RS2_FORMAT_XYZ32F:
        cv_type = CV_32FC3;
        break;
    // 32-bit y0, u, y1, v data for every two pixels. Similar to YUV422
    // but packed in a different order -
    // https://en.wikipedia.org/wiki/YUV
    case RS2_FORMAT_YUYV:
        break;
    // 8-bit red, green and blue channels
    case RS2_FORMAT_RGB8:
        cv_type = CV_8UC3;
        break;
    // 8-bit blue, green, and red channels – suitable for OpenCV
    case RS2_FORMAT_BGR8:
        cv_type = CV_8UC3;
        break;
    // 8-bit red, green and blue channels + constant alpha channel equal to FF
    case RS2_FORMAT_RGBA8:
        cv_type = CV_8UC4;
        break;
    // 8-bit blue, green, and red channels + constant alpha channel equal to FF
    case RS2_FORMAT_BGRA8:
        cv_type = CV_8UC4;
        break;
    // 8-bit per-pixel grayscale image
    case RS2_FORMAT_Y8:
        cv_type = CV_8UC1;
        break;
    // 16-bit per-pixel grayscale image
    case RS2_FORMAT_Y16:
        cv_type = CV_16UC1;
        break;
    // Four 10 bits per pixel luminance values packed into a 5-byte macropixel
    case RS2_FORMAT_RAW10:
        break;
    // 16-bit raw image
    case RS2_FORMAT_RAW16:
        cv_type = CV_16UC1;
        break;
    // 8-bit raw image
    case RS2_FORMAT_RAW8:
        cv_type = CV_8UC1;
        break;
    // Similar to the standard YUYV pixel format, but packed in a different order
    case RS2_FORMAT_UYVY:
        break;
    // Raw data from the motion sensor
    case RS2_FORMAT_MOTION_RAW:
        break;
    // Motion data packed as 3 32-bit float values, for X, Y, and Z axis
    case RS2_FORMAT_MOTION_XYZ32F:
        cv_type = CV_32FC3;
        break;
    // Raw data from the external sensors hooked to one of the GPIO's
    case RS2_FORMAT_GPIO_RAW:
        break;
    // Pose data packed as floats array, containing translation vector,
    // rotation quaternion and prediction velocities and accelerations
    // vectors
    case RS2_FORMAT_6DOF:
        break;
    // 32-bit float-point disparity values. Depth->Disparity conversion
    // : Disparity = Baseline*FocalLength/Depth
    case RS2_FORMAT_DISPARITY32:
        cv_type = CV_32FC1;
        break;
    // 16-bit per-pixel grayscale image unpacked from 10 bits per pixel
    // packed ([8:8:8:8:2222]) grey-scale image. The data is unpacked to
    // LSB and padded with 6 zero bits
    case RS2_FORMAT_Y10BPACK:
        break;
    // 32-bit float-point depth distance value.
    case RS2_FORMAT_DISTANCE:
        cv_type = CV_32FC1;
        break;
    // Bitstream encoding for video in which an image of each frame is
    // encoded as JPEG-DIB
    case RS2_FORMAT_MJPEG:
        break;
    // Number of enumeration values. Not a valid input: intended to be
    // used in for-loops.
    case RS2_FORMAT_COUNT:
        break;
    default:
        break;
    }

    if (cv_type == -1) {
        throw std::runtime_error("Unsupported image data type");
    }

    return cv::Mat(
            cv::Size(frame.get_width(), frame.get_height()),
            cv_type,
            const_cast<void*>(frame.get_data())
            );
}

} // anonymous namespace

namespace CameraSlam {

RealSenseCamera::RealSenseCamera(
        const std::string& config_file_path,
        const std::string& input_file_path,
        const std::string& output_file_path
        )
    : StereoCamera(config_file_path, input_file_path, output_file_path)
    , _pipe {}
    , _frame_left {rs2::frame()}
    , _frame_right {rs2::frame()}
    , _rgb {rs2::frame()}
    , _depth {rs2::frame()}
{
    if (_playback && _recording) {
        throw std::runtime_error("Cannot record and play at the same time");
    }

    _yaml_node["Camera.name"] = "RealSense D435";
    _yaml_node["Camera.model"] = "perspective";

    if (_rgbd_mode) {
        _yaml_node["Camera.color_order"] = "RGB";
    }
    else {
        _yaml_node["Camera.color_order"] = "Gray";
    }

    // Start pipeline
    rs2::pipeline_profile pipe_profile = _pipe.start(_make_realsense_config());

    _depth_scale = pipe_profile.get_device().query_sensors().front().as<rs2::depth_sensor>().get_depth_scale();

    // Turn off IR projector
    if (!_playback && !_rgbd_mode) {
        for (auto const& element : pipe_profile.get_device().query_sensors()) {
            if (strcmp(element.get_info(RS2_CAMERA_INFO_NAME), "Stereo Module") == 0) {
                element.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, 0u);
            }
        }
    }

    _read_camera_calibration();
}

void RealSenseCamera::cleanup(void)
{
    _pipe.stop();
}

bool RealSenseCamera::grab(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& rgb, cv::Mat& depth, double& timestamp)
{
    rs2::frameset frameset = _pipe.wait_for_frames();

    if (_rgbd_mode) {
        // Align depth to RGB
        rs2::align align {RS2_STREAM_COLOR};
        auto const& processed = align.process(frameset);
        _rgb = processed.get_color_frame();
        _depth = processed.get_depth_frame();

        if (!_rgb || !_depth) {
            return false;
        }

        timestamp = static_cast<double>(_rgb.get_timestamp()) / 1e3;
        rgb = frame_to_cv(_rgb);
        frame_to_cv(_depth).convertTo(depth, CV_32F);
        depth *= _depth_scale;
    }
    else {
        _frame_left = frameset.get_infrared_frame(StreamIndex::Left);
        _frame_right = frameset.get_infrared_frame(StreamIndex::Right);

        if (!_frame_left || !_frame_right) {
            return false;
        }

        timestamp = static_cast<double>(_frame_left.get_timestamp()) / 1e3;
        frame_left = frame_to_cv(_frame_left);
        frame_right = frame_to_cv(_frame_right);
    }
    return true;
}

bool RealSenseCamera::has_frames(void)
{
    if (_playback) {
        auto const& device = _pipe.get_active_profile().get_device().as<rs2::playback>();
        auto position = device.get_position();
        auto duration = static_cast<uint64_t>(device.get_duration().count());
        return position < duration;
    }
    return true;
}

rs2::config RealSenseCamera::_make_realsense_config(void)
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
        config.enable_stream(RS2_STREAM_COLOR, -1, cols, rows, RS2_FORMAT_RGB8, fps);
        config.enable_stream(RS2_STREAM_DEPTH, -1, cols, rows, RS2_FORMAT_ANY, fps);
        config.enable_stream(RS2_STREAM_INFRARED, StreamIndex::Left, cols, rows, RS2_FORMAT_Y8, fps);
        config.enable_stream(RS2_STREAM_INFRARED, StreamIndex::Right, cols, rows, RS2_FORMAT_Y8, fps);
    }

    if (_recording) {
        config.enable_record_to_file(_output_file_path);
    }

    return config;
}

void RealSenseCamera::_read_camera_calibration(void)
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
}

} // namespace CameraSlam

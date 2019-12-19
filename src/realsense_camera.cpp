#include "realsense_camera.hpp"

#include <opencv2/calib3d.hpp>

// Dependencies are not lint-clean
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif // defined(__GNUC__) || defined(__clang__)

#include <spdlog/spdlog.h>

#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic pop
#endif // defined(__GNUC__) || defined(__clang__)

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

template<typename T>
YAML::Node ptr_to_node(const void *ptr, const size_t len)
{
    return YAML::Node {std::vector<T>((T*) ptr, (T*) ptr + len)};
}

} // anonymous namespace

namespace YAML {

template<>
struct convert<cv::Mat> {
    static Node encode(const cv::Mat& rhs) {
        void *ptr = rhs.isContinuous() ? rhs.data : rhs.clone().data;
        size_t length = rhs.total() * rhs.channels();
        switch (rhs.type() & CV_MAT_DEPTH_MASK) {
            case CV_8U:
                return ptr_to_node<unsigned char>(ptr, length);
            case CV_8S:
                return ptr_to_node<char>(ptr, length);
            case CV_16U:
                return ptr_to_node<unsigned short>(ptr, length);
            case CV_16S:
                return ptr_to_node<short>(ptr, length);
            case CV_32S:
                return ptr_to_node<int>(ptr, length);
            case CV_32F:
                return ptr_to_node<float>(ptr, length);
            case CV_64F:
                return ptr_to_node<double>(ptr, length);
            default:
                throw std::runtime_error("Unsupported type");
        }
    }
};

template<>
struct convert<cv::Vec4d> {
    static Node encode(const cv::Vec4d& rhs) {
        Node node;
        node.push_back(rhs[0]);
        node.push_back(rhs[1]);
        node.push_back(rhs[2]);
        node.push_back(rhs[3]);
        return node;
    }
};

}

namespace CameraSlam {

RealSenseCamera::RealSenseCamera(
        const std::string& config_file_path,
        const std::string& input_file_path,
        const std::string& output_file_path
        )
    : StereoCamera(config_file_path, input_file_path, output_file_path)
    , _pipe (_context)
    , _frame_left {rs2::frame()}
    , _frame_right {rs2::frame()}
    , _rgb {rs2::frame()}
    , _depth {rs2::frame()}
{
    if (_playback && _recording) {
        throw std::runtime_error("Cannot record and play at the same time");
    }

    _yaml_node["Camera.name"] = "RealSense";
    _yaml_node["Camera.model"] = "perspective";

    if (_mode == Mode::RGBD) {
        _yaml_node["Camera.color_order"] = "RGB";
        _yaml_node["Camera.setup"] = "RGBD";
    }
    else {
        _yaml_node["Camera.color_order"] = "Gray";
        _yaml_node["Camera.setup"] = "stereo";
    }

    if (!_playback) {
        for (auto const& sensor : _context.query_all_sensors()) {
            // Stereo camera options
            if (sensor.supports(rs2_option::RS2_OPTION_EMITTER_ENABLED)) {
                _depth_scale = sensor.as<rs2::depth_sensor>().get_depth_scale();

                // Turn off IR projector
                sensor.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, ((_mode == Mode::PERSPECTIVE) ? 0u : 1u));
                spdlog::info("Disabling IR emitter");
            }
            // Tracking camera options
            else {
                if (sensor.supports(rs2_option::RS2_OPTION_ENABLE_RELOCALIZATION)) {
                    spdlog::info("Disabling RealSense relocalization");
                    sensor.set_option(rs2_option::RS2_OPTION_ENABLE_RELOCALIZATION, 0u);
                }
            }
        }
    }

    // Start pipeline
    rs2::pipeline_profile pipe_profile = _pipe.start(_make_realsense_config());

    _read_camera_calibration();
}

void RealSenseCamera::cleanup(void)
{
    _pipe.stop();
}

bool RealSenseCamera::grab(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& rgb, cv::Mat& depth, double& timestamp)
{
    rs2::frameset frameset = _pipe.wait_for_frames();

    if (_mode == Mode::RGBD) {
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
        if (_mode == Mode::FISHEYE) {
            _frame_left = frameset.get_fisheye_frame(StreamIndex::Left);
            _frame_right = frameset.get_fisheye_frame(StreamIndex::Right);
        }
        else {
            _frame_left = frameset.get_infrared_frame(StreamIndex::Left);
            _frame_right = frameset.get_infrared_frame(StreamIndex::Right);
        }

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
    if (_mode == Mode::FISHEYE) {
        _yaml_node["Camera.rows"] = T265.rows;
        _yaml_node["Camera.cols"] = T265.cols;
        _yaml_node["Camera.fps"] = T265.fps;
    }

    // Set camera settings
    auto const rows = _yaml_node["Camera.rows"].as<int>();
    auto const cols = _yaml_node["Camera.cols"].as<int>();
    auto const fps = static_cast<int>(std::round(_yaml_node["Camera.fps"].as<float>()));

    rs2::config config;

    if (_playback) {
        config.enable_device_from_file(_input_file_path);
    }
    else {
        switch (_mode) {
        case Mode::FISHEYE:
            config.enable_stream(RS2_STREAM_FISHEYE, StreamIndex::Left, cols, rows, T265.format, fps);
            config.enable_stream(RS2_STREAM_FISHEYE, StreamIndex::Right, cols, rows, T265.format, fps);
            break;
        case Mode::RGBD:
            config.enable_stream(RS2_STREAM_COLOR, -1, cols, rows, RS2_FORMAT_RGB8, fps);
            config.enable_stream(RS2_STREAM_DEPTH, -1, cols, rows, RS2_FORMAT_ANY, fps);
            break;
        case Mode::PERSPECTIVE:
            config.enable_stream(RS2_STREAM_INFRARED, StreamIndex::Left, cols, rows, RS2_FORMAT_Y8, fps);
            config.enable_stream(RS2_STREAM_INFRARED, StreamIndex::Right, cols, rows, RS2_FORMAT_Y8, fps);
            break;
        default:
            throw std::runtime_error("Unsupported mode");
        }
    }

    if (_recording) {
        config.enable_record_to_file(_output_file_path);
    }

    return config;
}

void RealSenseCamera::_read_camera_calibration(void)
{
    auto const intrinsics_left = _get_intrinsics(StreamIndex::Left);

    if (_mode == Mode::FISHEYE) {
        auto const intrinsics_right = _get_intrinsics(StreamIndex::Right);
        auto const extrinsics = _get_extrinsics();

        const cv::Mat camera_matrix_left = (cv::Mat_<double>(3,3) <<
            intrinsics_left.fx,  0.0f,                intrinsics_left.ppx,
            0.0f,                intrinsics_left.fy,  intrinsics_left.ppy,
            0.0f,                0.0f,                1.0f
        );

        const cv::Mat camera_matrix_right = (cv::Mat_<double>(3,3) <<
            intrinsics_right.fx, 0.0f,                intrinsics_right.ppx,
            0.0f,                intrinsics_right.fy, intrinsics_right.ppy,
            0.0f,                0.0f,                1.0f
        );

        const cv::Vec4d distortion_left {
            intrinsics_left.coeffs[0],
            intrinsics_left.coeffs[1],
            intrinsics_left.coeffs[2],
            intrinsics_left.coeffs[3],
        };

        const cv::Vec4d distortion_right {
            intrinsics_right.coeffs[0],
            intrinsics_right.coeffs[1],
            intrinsics_right.coeffs[2],
            intrinsics_right.coeffs[3],
        };

        const cv::Size size {T265.cols, T265.rows};

        const cv::Mat rotation = (cv::Mat_<double>(3,3) <<
            extrinsics.rotation[0], extrinsics.rotation[3], extrinsics.rotation[6],
            extrinsics.rotation[1], extrinsics.rotation[4], extrinsics.rotation[7],
            extrinsics.rotation[2], extrinsics.rotation[5], extrinsics.rotation[8]
        );

        const cv::Vec3d translation {
            extrinsics.translation[0],
            extrinsics.translation[1],
            extrinsics.translation[2],
        };

        // Rectification output
        cv::Mat rectification_matrix_left,
                rectification_matrix_right,
                projection_matrix_left,
                projection_matrix_right,
                disparity_to_depth_matrix;

        // Rectify
        cv::fisheye::stereoRectify(
                camera_matrix_left,
                distortion_left,
                camera_matrix_right,
                distortion_right,
                size,
                rotation,
                translation,
                rectification_matrix_left,
                rectification_matrix_right,
                projection_matrix_left,
                projection_matrix_right,
                disparity_to_depth_matrix,
                0
                );

        _yaml_node["Camera.fx"] = projection_matrix_left.at<double>(0, 0);
        _yaml_node["Camera.fy"] = projection_matrix_left.at<double>(1, 1);
        _yaml_node["Camera.cx"] = projection_matrix_left.at<double>(0, 2);
        _yaml_node["Camera.cy"] = projection_matrix_left.at<double>(1, 2);

        _yaml_node["Camera.focal_x_baseline"] = projection_matrix_right.at<double>(0, 3);

        _yaml_node["Camera.k1"] = 0.0f;
        _yaml_node["Camera.k2"] = 0.0f;
        _yaml_node["Camera.p1"] = 0.0f;
        _yaml_node["Camera.p2"] = 0.0f;
        _yaml_node["Camera.k3"] = 0.0f;

        _yaml_node["StereoRectifier.model"] = "fisheye";
        _yaml_node["StereoRectifier.K_left"] = camera_matrix_left;
        _yaml_node["StereoRectifier.D_left"] = distortion_left;
        _yaml_node["StereoRectifier.R_left"] = rectification_matrix_left;
        _yaml_node["StereoRectifier.K_right"] = camera_matrix_right;
        _yaml_node["StereoRectifier.D_right"] = distortion_right;
        _yaml_node["StereoRectifier.R_right"] = rectification_matrix_right;
    }
    else {
        _yaml_node["Camera.fx"] = intrinsics_left.fx;
        _yaml_node["Camera.fy"] = intrinsics_left.fy;
        _yaml_node["Camera.cx"] = intrinsics_left.ppx;
        _yaml_node["Camera.cy"] = intrinsics_left.ppy;

        _yaml_node["Camera.k1"] = intrinsics_left.coeffs[0];
        _yaml_node["Camera.k2"] = intrinsics_left.coeffs[1];
        _yaml_node["Camera.p1"] = intrinsics_left.coeffs[2];
        _yaml_node["Camera.p2"] = intrinsics_left.coeffs[3];
        _yaml_node["Camera.k3"] = intrinsics_left.coeffs[4];
    }

    if (_mode == Mode::PERSPECTIVE) {
        auto const extrinsics = _get_extrinsics();
        _yaml_node["Camera.focal_x_baseline"] = extrinsics.translation[0] * intrinsics_left.fx;
    }
}

rs2_stream RealSenseCamera::_get_stream_type(void)
{
    switch (_mode) {
    case Mode::FISHEYE:
        return RS2_STREAM_FISHEYE;
    case Mode::RGBD:
        return RS2_STREAM_COLOR;
    case Mode::PERSPECTIVE:
        return RS2_STREAM_INFRARED;
    default:
        throw std::runtime_error("Unsupported mode");
    }
}

rs2_intrinsics RealSenseCamera::_get_intrinsics(const StreamIndex stream_index)
{
    auto const left_stream = _pipe.get_active_profile().get_stream(_get_stream_type(), stream_index).as<rs2::video_stream_profile>();
    return left_stream.get_intrinsics();
}

rs2_extrinsics RealSenseCamera::_get_extrinsics(void)
{
    auto const left_stream = _pipe.get_active_profile().get_stream(_get_stream_type(), StreamIndex::Left).as<rs2::video_stream_profile>();
    auto const right_stream = _pipe.get_active_profile().get_stream(_get_stream_type(), StreamIndex::Right).as<rs2::video_stream_profile>();
    return right_stream.get_extrinsics_to(left_stream);
}

} // namespace CameraSlam

#include "zed_camera.hpp"

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

/**
 * \brief Conversion function between sl::Mat and cv::Mat.
 * From https://github.com/stereolabs/zed-opencv/blob/da64dd8c1f2/cpp/src/main.cpp#L107-L128
 *
 * \note The `frame` object retains ownership of the image buffer.
 **/
cv::Mat slMat2cvMat(sl::Mat &input) {
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

} // anonymous namespace

namespace CameraSlam {

ZedCamera::ZedCamera(
        const std::string& config_file_path,
        const std::string& input_svo_file_path,
        const std::string& output_svo_file_path
        )
    : StereoCamera(config_file_path, input_svo_file_path, output_svo_file_path)
{
    _yaml_node["Camera.name"] = "ZED";
    _yaml_node["Camera.model"] = "perspective";
    _yaml_node["Camera.color_order"] = "RGB";

    auto param = _make_zed_parameters();

    // Playback from file
    if (!_input_file_path.empty()) {
        param.input.setFromSVOFile(sl::String(_input_file_path.data()));
        spdlog::info("Playback from file {0}", _input_file_path);
    }

    // Open the camera
    auto const err = _zed.open(param);
    if (err != sl::ERROR_CODE::SUCCESS) {
        _zed.close();
        throw std::runtime_error(toString(err));
    }

    // Enable recording mode
    if (!_output_file_path.empty()) {
        const sl::RecordingParameters recording_parameters (sl::String(_output_file_path.data()), sl::SVO_COMPRESSION_MODE::LOSSLESS);
        auto const err = _zed.enableRecording(recording_parameters);
        if (err != sl::ERROR_CODE::SUCCESS) {
            throw std::runtime_error("Cannot open output file " + _output_file_path);
        }
    }

    // Read camera calibration
    _read_zed_camera_calibration();
}

void ZedCamera::cleanup(void)
{
    _zed.close();
}

bool ZedCamera::grab(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& rgb, cv::Mat& depth, double& timestamp)
{
    if (_zed.grab() != sl::ERROR_CODE::SUCCESS) {
        return false;
    }

    if (!_output_file_path.empty() && !_zed.getRecordingStatus().status) {
        spdlog::info("Error writing frame to {0}", _output_file_path);
    }

    if ((_zed.retrieveImage(_frame_left, sl::VIEW::LEFT_GRAY) != sl::ERROR_CODE::SUCCESS) ||
        (_zed.retrieveImage(_frame_right, sl::VIEW::RIGHT_GRAY) != sl::ERROR_CODE::SUCCESS) ||
        (_zed.retrieveImage(_rgb, sl::VIEW::LEFT) != sl::ERROR_CODE::SUCCESS) ||
        (_zed.retrieveImage(_depth, sl::VIEW::DEPTH) != sl::ERROR_CODE::SUCCESS)) {
        return false;
    }

    frame_left = slMat2cvMat(_frame_left);
    frame_right = slMat2cvMat(_frame_right);
    rgb = slMat2cvMat(_rgb);
    depth = slMat2cvMat(_depth);
    timestamp = _timestamp();
    return true;
}

bool ZedCamera::has_frames(void)
{
    return !_input_file_path.empty() && (_zed.getSVOPosition() < (_zed.getSVONumberOfFrames() - 1));
}

double ZedCamera::_timestamp(void)
{
    return static_cast<double>(_zed.getTimestamp(sl::TIME_REFERENCE::IMAGE)) / 1e9;
}

sl::InitParameters ZedCamera::_make_zed_parameters(void)
{
    sl::InitParameters param;

    // Set camera settings
    auto const rows = _yaml_node["Camera.rows"].as<int>();
    auto const cols = _yaml_node["Camera.cols"].as<int>();
    auto const fps = static_cast<int>(std::round(_yaml_node["Camera.fps"].as<float>()));
    bool invalid_fps = false;
    if (cols == 376 && rows == 672) {
        param.camera_resolution = sl::RESOLUTION::VGA;
        invalid_fps = (fps != 15 && fps != 30 && fps != 60 && fps != 100);
    }
    else if (cols == 1280 && rows == 720) {
        param.camera_resolution = sl::RESOLUTION::HD720;
        invalid_fps = (fps != 15 && fps != 30 && fps != 60);
    }
    else if (cols == 1920 && rows == 1080) {
        param.camera_resolution = sl::RESOLUTION::HD1080;
        invalid_fps = (fps != 15 && fps != 30);
    }
    else if (cols == 2208 && rows == 1242) {
        param.camera_resolution = sl::RESOLUTION::HD2K;
        invalid_fps = (fps != 15);
    }
    else {
        throw std::runtime_error(
                "Unsupported resolution " + std::to_string(rows) + "x" + std::to_string(cols));
    }
    if (invalid_fps) {
        throw std::runtime_error(
                "Unsupported frame rate " + std::to_string(fps)
                + " at resolution" + std::to_string(rows) + "x" + std::to_string(cols));
    }
    param.camera_fps = fps;
    param.coordinate_units = sl::UNIT::METER;

    return param;
}

void ZedCamera::_read_zed_camera_calibration(void)
{
    const auto rectified_calibration = _zed.getCameraInformation().calibration_parameters;

    _yaml_node["Camera.fx"] = rectified_calibration.left_cam.fx;
    _yaml_node["Camera.fy"] = rectified_calibration.left_cam.fy;
    _yaml_node["Camera.cx"] = rectified_calibration.left_cam.cx;
    _yaml_node["Camera.cy"] = rectified_calibration.left_cam.cy;
    _yaml_node["Camera.k1"] = rectified_calibration.left_cam.disto[0];
    _yaml_node["Camera.k2"] = rectified_calibration.left_cam.disto[1];
    _yaml_node["Camera.p1"] = rectified_calibration.left_cam.disto[2];
    _yaml_node["Camera.p2"] = rectified_calibration.left_cam.disto[3];
    _yaml_node["Camera.k3"] = rectified_calibration.left_cam.disto[4];
    _yaml_node["Camera.focal_x_baseline"] = rectified_calibration.T.x * rectified_calibration.left_cam.fx;
}

} // namespace CameraSlam

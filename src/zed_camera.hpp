#include "stereo_camera.hpp"

// Dependencies are not lint-clean
#if defined(__GNUC__) || defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    #pragma GCC diagnostic ignored "-Wreorder"
    #pragma GCC diagnostic ignored "-Wignored-qualifiers"
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif // defined(__GNUC__) || defined(__clang__)

#include <sl/Camera.hpp>
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
cv::Mat slMat2cvMat(sl::Mat& input)
{
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(
            static_cast<int>(input.getHeight()),
            static_cast<int>(input.getWidth()),
            cv_type,
            input.getPtr<sl::uchar1>(sl::MEM_CPU)
            );
}

} // anonymous namespace

class ZedCamera : public StereoCamera {

public:
    explicit ZedCamera(
            const std::string& config_file_path,
            const std::string& input_svo_file_path,
            const std::string& output_svo_file_path
            )
        : StereoCamera(config_file_path, input_svo_file_path, output_svo_file_path)
    {
        _yaml_node["Camera.name"] = "ZED";
        _yaml_node["Camera.setup"] = "stereo";
        _yaml_node["Camera.model"] = "perspective";
        _yaml_node["Camera.color_order"] = "RGB";

        auto param = _make_zed_parameters();

        // Playback from file
        if (!_input_file_path.empty()) {
            param.svo_input_filename = sl::String(_input_file_path.data());
            spdlog::info("Playback from file {0}", _input_file_path);
        }

        // Open the camera
        auto const err = _zed.open(param);
        if (err != sl::SUCCESS) {
            _zed.close();
            throw std::runtime_error(toString(err));
        }

        // Enable recording mode
        if (!_output_file_path.empty()) {
            auto const err = _zed.enableRecording(sl::String(_output_file_path.data()), sl::SVO_COMPRESSION_MODE_LOSSLESS);
            if (err != sl::SUCCESS) {
                throw std::runtime_error("Cannot open output file " + _output_file_path);
            }
        }

        // Read camera calibration
        _read_zed_camera_calibration();
    }

    virtual void cleanup(void) override
    {
        _zed.close();
    }

    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp) override
    {
        if (_zed.grab() != sl::SUCCESS) {
            return false;
        }

        if (!_output_file_path.empty()) {
            auto const rs = _zed.record();
            if (!rs.status) {
                throw std::runtime_error("Error writing frame to " + _output_file_path);
            }
        }

        if (_zed.retrieveImage(_frame_left, sl::VIEW_LEFT_GRAY) != sl::SUCCESS ||
            _zed.retrieveImage(_frame_right, sl::VIEW_RIGHT_GRAY) != sl::SUCCESS) {
            return false;
        }

        frame_left = slMat2cvMat(_frame_left);
        frame_right = slMat2cvMat(_frame_right);
        timestamp = _timestamp();
        return true;
    }

    virtual bool has_frames(void) override
    {
        return !_input_file_path.empty() && (_zed.getSVOPosition() < (_zed.getSVONumberOfFrames() - 1));
    }

private:
    double _timestamp(void)
    {
        return static_cast<double>(_zed.getTimestamp(sl::TIME_REFERENCE_IMAGE)) / 1e9;
    }

    sl::InitParameters _make_zed_parameters(void)
    {
        sl::InitParameters param;

        // Set camera settings
        auto const rows = _yaml_node["Camera.rows"].as<int>();
        auto const cols = _yaml_node["Camera.cols"].as<int>();
        auto const fps = static_cast<int>(std::round(_yaml_node["Camera.fps"].as<float>()));
        bool invalid_fps = false;
        if (cols == 376 && rows == 672) {
            param.camera_resolution = sl::RESOLUTION_VGA;
            invalid_fps = (fps != 15 && fps != 30 && fps != 60 && fps != 100);
        }
        else if (cols == 1280 && rows == 720) {
            param.camera_resolution = sl::RESOLUTION_HD720;
            invalid_fps = (fps != 15 && fps != 30 && fps != 60);
        }
        else if (cols == 1920 && rows == 1080) {
            param.camera_resolution = sl::RESOLUTION_HD1080;
            invalid_fps = (fps != 15 && fps != 30);
        }
        else if (cols == 2208 && rows == 1242) {
            param.camera_resolution = sl::RESOLUTION_HD2K;
            invalid_fps = (fps != 15);
        }
        else {
            throw std::runtime_error("Unsupported resolution " + std::to_string(rows) + "x" + std::to_string(cols));
        }
        if (invalid_fps) {
            throw std::runtime_error("Unsupported frame rate " + std::to_string(fps) + " at resolution" + std::to_string(rows) + "x" + std::to_string(cols));
        }
        param.camera_fps = fps;
        param.coordinate_units = sl::UNIT_METER;

        return param;
    }

    void _read_zed_camera_calibration(void)
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

    sl::Camera _zed;
    sl::Mat _frame_left;
    sl::Mat _frame_right;
};

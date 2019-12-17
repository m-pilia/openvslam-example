#include "mynteye_camera.hpp"

#include <opencv2/imgproc.hpp>

namespace CameraSlam {

MyntEyeCamera::TimedImage::TimedImage(const cv::Mat& img, const uint64_t ts)
        : image{img}
        , timestamp{ts}
    {}

MyntEyeCamera::MyntEyeCamera(
        const std::string& config_file_path,
        const std::string& input_file_path,
        const std::string& output_file_path
        )
    : StereoCamera(config_file_path, input_file_path, output_file_path)
    , _api {_playback ? nullptr : mynteye::API::Create(0, {})}
    , _input_file {_playback ? std::make_unique<std::ifstream>(_input_file_path, std::ios::binary) : nullptr}
    , _output_file {_recording ? std::make_unique<std::ofstream>(_output_file_path, std::ios::binary) : nullptr}
    , _has_frames {true}
{
    if (!_playback && !_api) {
        throw std::runtime_error("Failed to create MyntEye API");
    }

    _yaml_node["Camera.name"] = "MYNT EYE S1030-IR";
    _yaml_node["Camera.model"] = "perspective";
    _yaml_node["Camera.color_order"] = "Gray";
    _yaml_node["Camera.rows"] = _stream_request.height;
    _yaml_node["Camera.cols"] = _stream_request.width;
    _yaml_node["Camera.fps"] = _stream_request.fps;

    if (_playback) {
        _read_header();
    }

    if (!_playback) {
        _read_camera_calibration();
    }

    if (_recording) {
        _write_header();
    }

    if (!_playback) {
        _api->ConfigStreamRequest(_stream_request);

        // Get DISPARITY stream instead of DEPTH stream
        // The DEPTH stream seems to have poor quality
        _api->EnableStreamData(mynteye::Stream::LEFT);
        _api->EnableStreamData(mynteye::Stream::RIGHT);
        _api->EnableStreamData(mynteye::Stream::LEFT_RECTIFIED);
        _api->EnableStreamData(mynteye::Stream::RIGHT_RECTIFIED);
        _api->EnableStreamData(mynteye::Stream::DISPARITY);

        // Turn off IR projector
        auto const&& ir_range = _api->GetOptionInfo(mynteye::Option::IR_CONTROL);
        _api->SetOptionValue(mynteye::Option::IR_CONTROL, ir_range.min);

        _api->Start(mynteye::Source::ALL);
    }
}

MyntEyeCamera::~MyntEyeCamera()
{
    cleanup();
}

void MyntEyeCamera::cleanup(void)
{
    if (!_playback) {
        _api->Stop(mynteye::Source::ALL);

        // It seems callbacks could still be running after _api->Stop()
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

bool MyntEyeCamera::grab(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& rgb, cv::Mat& depth, double& timestamp)
{
    uint64_t ts;

    if (_playback) {
        DataType data_type;
        if (!_read_datum(frame_left, ts, data_type) || data_type != DataType::LEFT ||
            !_read_datum(frame_right, ts, data_type) || data_type != DataType::RIGHT ||
            !_read_datum(depth, ts, data_type) || data_type != DataType::DEPTH) {
            _has_frames = false;
            return false;
        }
    }
    else {
        _api->WaitForStreams();

        _left_data = _api->GetStreamData(mynteye::Stream::LEFT_RECTIFIED);
        _right_data = _api->GetStreamData(mynteye::Stream::RIGHT_RECTIFIED);
        _depth_data = _api->GetStreamData(mynteye::Stream::DISPARITY);

        if (_left_data.frame.empty() || _right_data.frame.empty() || _depth_data.frame.empty()) {
            return false;
        }

        frame_left = _left_data.frame;
        frame_right = _right_data.frame;
        depth = _focal_baseline / _depth_data.frame;
        cv::cvtColor(frame_left, rgb, cv::COLOR_GRAY2RGB);
        ts = _left_data.img->timestamp;

        if (_recording) {
            _write_to_file(frame_left, _left_data.img->timestamp, DataType::LEFT);
            _write_to_file(frame_right, _left_data.img->timestamp, DataType::RIGHT);
            _write_to_file(depth, _left_data.img->timestamp, DataType::DEPTH);
        }
    }

    timestamp = static_cast<double>(ts) / 1e6;
    return true;
}

bool MyntEyeCamera::has_frames(void)
{
    return _has_frames;
}

/*!
 * \brief Read file header.
 *
 * \see MyntEyeCamera::_write_header(void)
 */
void MyntEyeCamera::_read_header(void)
{
    if (!_input_file || !_input_file->is_open()) {
        std::runtime_error("No file to read");
    }

    size_t header_size;
    _input_file->read(reinterpret_cast<char*>(&header_size), sizeof (size_t));

    std::vector<char> data (header_size);
    _input_file->read(reinterpret_cast<char*>(data.data()), header_size);

    auto const node = YAML::Load(std::string(data.data()));

    _yaml_node["Camera.focal_x_baseline"] = node["Camera.focal_x_baseline"].as<double>();
}

/*!
 * \brief Write file header.
 *
 * Write the header of the file. The file header starts at the beginning
 * of the file and has the format
 * <tt>
 *      <HEADER_SIZE><HEADER_DATA>
 * </tt>
 * where `HEADER_SIZE` is a `size_t` denoting the size in byte of `HEADER_DATA`,
 * and `HEADER_DATA` is a YAML string.
 */
void MyntEyeCamera::_write_header(void)
{
    if (!_output_file || !_output_file->is_open()) {
        throw std::runtime_error("No file to write");
    }

    YAML::Emitter out;
    out << YAML::BeginMap
        << YAML::Key << "Camera.focal_x_baseline" << YAML::Value << _yaml_node["Camera.focal_x_baseline"].as<double>()
        << YAML::EndMap;
    size_t const size = strlen(out.c_str());

    {
        std::scoped_lock lock {_mtx_file};
        _output_file->write(reinterpret_cast<const char*>(&size), sizeof (size_t));
        _output_file->write(out.c_str(), size);
    }
}

/*!
 * \brief Read calibration from the camera.
 */
void MyntEyeCamera::_read_camera_calibration(void)
{
    if (_playback) {
        throw std::runtime_error("Cannot access the camera in playback mode");
    }

    auto const extrinsics = _api->GetExtrinsics(mynteye::Stream::LEFT, mynteye::Stream::RIGHT);
    _focal_baseline = 1e-3 * extrinsics.translation[0] * _yaml_node["Camera.fx"].as<double>();
    _yaml_node["Camera.focal_x_baseline"] = _focal_baseline;
}

/*!
 * \brief Write image to file.
 *
 * \see MyntEyeCamera::_read_datum(cv::Mat& image, uint64_t& timestamp, DataType& data_type)
 */
void MyntEyeCamera::_write_to_file(const cv::Mat& image, const uint64_t timestamp, const DataType data_type)
{
    if (!_output_file || !_output_file->is_open()) {
        throw std::runtime_error("No file to write");
    }

    std::vector<unsigned char> output;
    cv::imencode(".png", image, output, {cv::IMWRITE_PNG_COMPRESSION, 0});
    const size_t size = output.size();

    {
        std::scoped_lock lock {_mtx_file};
        _output_file->write(reinterpret_cast<const char*>(&data_type), sizeof (DataType));
        _output_file->write(reinterpret_cast<const char*>(&timestamp), sizeof (uint64_t));
        _output_file->write(reinterpret_cast<const char*>(&size), sizeof (size_t));
        _output_file->write(reinterpret_cast<const char*>(output.data()), size * sizeof (unsigned char));
    }
}

/*!
 * \brief Read the next chunk (image or IMU) from file.
 *
 * Read a datum (an image or an IMU read) from file. The file has binary
 * encoding and contains a header (MyntEyeCamera::_read_header(void))
 * followed by a sequence of chunks. Each chunk is a datum and has the
 * form
 * <tt>
 *      <SIZE><DATA>
 * </tt>
 * where `SIZE` is a `size_t` denoting the size in byte of `DATA`.
 */
bool MyntEyeCamera::_read_datum(cv::Mat& image, uint64_t& timestamp, DataType& data_type)
{
    std::vector<unsigned char> input;
    size_t size;

    // Read from file
    {
        std::scoped_lock lock {_mtx_file};

        if (!_input_file || !_input_file->is_open()) {
            throw std::runtime_error("No file to read");
        }

        if (_input_file->eof()) {
            return false;
        }

        _input_file->read(reinterpret_cast<char*>(&data_type), sizeof (DataType));

        if (data_type != DataType::LEFT &&
            data_type != DataType::RIGHT &&
            data_type != DataType::DEPTH &&
            data_type != DataType::IMU) {
            return false;
        }

        _input_file->read(reinterpret_cast<char*>(&timestamp), sizeof (uint64_t));
        _input_file->read(reinterpret_cast<char*>(&size), sizeof (size_t));
        if (size < 1) {
            return false;
        }

        input.resize(size);
        _input_file->read(reinterpret_cast<char*>(input.data()), size * sizeof (unsigned char));
    }

    // Decode
    switch (data_type)
    {
        case DataType::LEFT:
            [[fallthrough]];
        case DataType::RIGHT:
            [[fallthrough]];
        case DataType::DEPTH:
            image = cv::imdecode(input, cv::IMREAD_GRAYSCALE);
            break;
        case DataType::IMU:
            throw std::runtime_error("NOT IMPLEMENTED");
    }

    return true;
}



} // namespace CameraSlam

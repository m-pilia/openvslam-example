#include "mynteye_camera.hpp"

#include <chrono>

#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;

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
    , _discard_old_image_thread {[&]() {_discard_old_image();}}
    , _is_running {true}
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

        // Set callbacks to handle data from the camera
        _api->SetStreamCallback(mynteye::Stream::LEFT_RECTIFIED, [&](auto data) {_handle_image(data, DataType::LEFT);});
        _api->SetStreamCallback(mynteye::Stream::RIGHT_RECTIFIED, [&](auto data) {_handle_image(data, DataType::RIGHT);});
        _api->SetStreamCallback(mynteye::Stream::DISPARITY, [&](auto data) {_handle_image(data, DataType::DEPTH);});

        // Turn off IR projector
        auto const&& ir_range = _api->GetOptionInfo(mynteye::Option::IR_CONTROL);
        _api->SetOptionValue(mynteye::Option::IR_CONTROL, ir_range.min);

        _api->Start(mynteye::Source::ALL);
    }
}


MyntEyeCamera::~MyntEyeCamera()
{
    _is_running = false;
    _discard_old_image_thread.join();
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
    if (_playback && !_read_image_frame()) {
        return false;
    }

    _dequeue_image_frame(frame_left, frame_right, depth, timestamp);
    cv::cvtColor(frame_left, rgb, cv::COLOR_GRAY2RGB);
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
 * \brief Add a datum (image or IMU) to the queue.
 */
void MyntEyeCamera::_enqueue(const cv::Mat& image, const uint64_t timestamp, const DataType data_type)
{
    {
        std::scoped_lock lock {_mtx_queue};
        switch (data_type)
        {
            case DataType::LEFT:
                _queue_left.emplace(image, timestamp);
                break;
            case DataType::RIGHT:
                _queue_right.emplace(image, timestamp);
                break;
            case DataType::DEPTH:
                _queue_depth.emplace(image, timestamp);
                break;
            case DataType::IMU:
                throw std::runtime_error("NOT IMPLEMENTED");
                break;
        }
    }
    _condition_variable_queue.notify_one();
    _condition_variable_discard.notify_one();
}

/*!
 * \brief Extract a stereo pair from the image queues.
 */
void MyntEyeCamera::_dequeue_image_frame(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& depth, double& timestamp)
{
    std::unique_lock lock {_mtx_queue};
    _condition_variable_queue.wait(lock, [&]() {return _frame_ready();});
    frame_left = _queue_left.front().image;
    frame_right = _queue_right.front().image;
    depth = _queue_depth.front().image;
    timestamp = static_cast<double>(_queue_right.front().timestamp) / 1e6;
    _queue_left.pop();
    _queue_right.pop();
    _queue_depth.pop();
}

/*!
 * \brief Callback to handle frame image data.
 */
void MyntEyeCamera::_handle_image(const mynteye::api::StreamData& data, const DataType data_type)
{
    if (data.frame.empty()) {
        return;
    }

    // Convert disparity to depth
    const auto& image = (data_type == DataType::DEPTH) ? (_focal_baseline / data.frame) : data.frame;

    // Enqueue image
    _enqueue(image, data.img->timestamp, data_type);

    // Write to file
    if (_recording) {
        _write_to_file(image, data.img->timestamp, data_type);
    }
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

/*!
 * \brief Read image pair from file.
 */
bool MyntEyeCamera::_read_image_frame(void)
{
    while (_frame_not_queued())
    {
        cv::Mat frame;
        uint64_t timestamp;
        DataType data_type;
        if (!_read_datum(frame, timestamp, data_type)) {
            _has_frames = false;
            return false;
        }

        _enqueue(frame, timestamp, data_type);
    }
    return true;
}

/*!
 * \brief Handle dropped frames.
 *
 * If a stereo image or depth is not matched with its corresponding ones
 * after `_queue_max_length` images were read, drop it.
 */
void MyntEyeCamera::_discard_old_image(void)
{
    while (_is_running) {
        std::unique_lock lock {_mtx_queue};
        _condition_variable_discard.wait_for(lock, 1s, [&]{return _is_long_queue();});

        bool not_empty = !(_queue_left.empty() || _queue_right.empty() || _queue_depth.empty());
        bool mismatching = (_queue_left.front().timestamp != _queue_right.front().timestamp) ||
                           (_queue_left.front().timestamp != _queue_depth.front().timestamp);

        while (not_empty && mismatching) {
            auto const cmp = [](auto const& l, auto const& r) {return l.front().timestamp < r.front().timestamp;};
            std::min({_queue_left, _queue_right, _queue_depth}, cmp).pop();
        }
    }
}

/*!
 * \brief Return `true` if all images of a frame are ready in the queues.
 *
 * This function is not thread safe. It can be called while the
 * `_mtx_queue` is already being hold.
 */
bool MyntEyeCamera::_frame_ready(void)
{
    bool not_ready = _queue_left.empty() || _queue_right.empty() || _queue_depth.empty() ||
        (_queue_left.front().timestamp != _queue_right.front().timestamp) ||
        (_queue_left.front().timestamp != _queue_depth.front().timestamp);
    return !not_ready;
}

/*!
 * \brief Return `true` if not all images of a frame are ready in the queues.
 *
 * This function is thread safe.
 */

bool MyntEyeCamera::_frame_not_queued(void)
{
    std::scoped_lock lock {_mtx_queue};
    return !_frame_ready();
}

/*!
 * \brief Return `true` if many frames are queued.
 */
bool MyntEyeCamera::_is_long_queue(void)
{
    return (_queue_left.size() > _queue_max_length) ||
        (_queue_right.size() > _queue_max_length) ||
        (_queue_depth.size() > _queue_max_length);
}

} // namespace CameraSlam

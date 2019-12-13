#pragma once

#include "stereo_camera.hpp"

#include <fstream>
#include <queue>
#include <tuple>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <mynteye/api/api.h>
#include <spdlog/spdlog.h>

class MyntEyeCamera : public StereoCamera
{
public:
    explicit MyntEyeCamera(
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
        _yaml_node["Camera.setup"] = "stereo";
        _yaml_node["Camera.model"] = "perspective";
        _yaml_node["Camera.color_order"] = "Gray";
        _yaml_node["Camera.rows"] = _stream_request.height;
        _yaml_node["Camera.cols"] = _stream_request.width;
        _yaml_node["Camera.fps"] = _stream_request.fps;

        _read_header();
        _read_camera_calibration();
        _write_header();

        if (!_playback) {
            _api->ConfigStreamRequest(_stream_request);

            _api->EnableStreamData(mynteye::Stream::LEFT);
            _api->EnableStreamData(mynteye::Stream::RIGHT);
            _api->EnableStreamData(mynteye::Stream::LEFT_RECTIFIED);
            _api->EnableStreamData(mynteye::Stream::RIGHT_RECTIFIED);

            if (_recording) {
                _api->SetStreamCallback(mynteye::Stream::LEFT_RECTIFIED, [&](auto data) {_write_image(data, DataType::LEFT);});
                _api->SetStreamCallback(mynteye::Stream::RIGHT_RECTIFIED, [&](auto data) {_write_image(data, DataType::RIGHT);});
            }

            _api->Start(mynteye::Source::ALL);
        }
    }

    virtual ~MyntEyeCamera()
    {
        cleanup();
    }

    virtual void cleanup(void)
    {
        if (!_playback) {
            _api->Stop(mynteye::Source::ALL);
        }
    }

    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp)
    {
        if (_playback) {
            return _read_image_pair(frame_left, frame_right, timestamp);
        }
        else {
            return _grab_image_pair(frame_left, frame_right, timestamp);
        }
    }

    virtual bool has_frames(void)
    {
        return _has_frames;
    }

private:
    enum DataType {LEFT, RIGHT, IMU};

    struct TimedImage {
        TimedImage(const cv::Mat& img, const uint64_t ts)
            : image{img}
            , timestamp{ts}
        {}
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

    void _read_header(void)
    {
        if (!_playback) {
            return;
        }

        size_t header_size;
        _input_file->read(reinterpret_cast<char*>(&header_size), sizeof (size_t));

        std::vector<char> data (header_size);
        _input_file->read(reinterpret_cast<char*>(data.data()), header_size);

        auto const node = YAML::Load(std::string(data.data()));

        _yaml_node["Camera.focal_x_baseline"] = node["Camera.focal_x_baseline"].as<double>();
    }

    void _write_header(void)
    {
        if (!_recording) {
            return;
        }

        YAML::Emitter out;
        out << YAML::BeginMap
            << YAML::Key << "Camera.focal_x_baseline" << YAML::Value << _yaml_node["Camera.focal_x_baseline"].as<double>()
            << YAML::EndMap;
        size_t const size = strlen(out.c_str());

        {
            std::scoped_lock lock {_mtx};
            _output_file->write(reinterpret_cast<const char*>(&size), sizeof (size_t));
            _output_file->write(out.c_str(), size);
        }
    }

    void _read_camera_calibration(void)
    {
        if (_playback) {
            return;
        }

        auto const extrinsics = _api->GetExtrinsics(mynteye::Stream::LEFT, mynteye::Stream::RIGHT);
        _yaml_node["Camera.focal_x_baseline"] = 1e-3 * extrinsics.translation[0] * _yaml_node["Camera.fx"].as<double>();
    }

    void _enqueue_image(const cv::Mat& image, const uint64_t timestamp, const DataType data_type)
    {
        switch (data_type)
        {
            case DataType::LEFT:
                _queue_left.emplace(image, timestamp);
                break;
            case DataType::RIGHT:
                _queue_right.emplace(image, timestamp);
                break;
            case DataType::IMU:
                throw std::runtime_error("NOT IMPLEMENTED");
                break;
        }
    }

    void _dequeue_image_pair(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp)
    {
        frame_left = _queue_left.front().image;
        frame_right = _queue_right.front().image;
        timestamp = static_cast<double>(_queue_right.front().timestamp) / 1e6;
        _queue_left.pop();
        _queue_right.pop();
    }

    void _write_image(const mynteye::api::StreamData& data, const DataType data_type)
    {
        if (!_output_file || !_output_file->is_open()) {
            throw std::runtime_error("No file to write");
        }

        if (data.frame.empty()) {
            return;
        }

        std::vector<unsigned char> output;
        cv::imencode(".png", data.frame, output, {cv::IMWRITE_PNG_COMPRESSION, 0});
        const size_t size = output.size();

        {
            std::scoped_lock lock {_mtx};
            _output_file->write(reinterpret_cast<const char*>(&data_type), sizeof (DataType));
            _output_file->write(reinterpret_cast<const char*>(&data.img->timestamp), sizeof (uint64_t));
            _output_file->write(reinterpret_cast<const char*>(&size), sizeof (size_t));
            _output_file->write(reinterpret_cast<const char*>(output.data()), size * sizeof (unsigned char));
        }
    }

    bool _read_datum(cv::Mat& image, uint64_t& timestamp, DataType& data_type)
    {
        std::vector<unsigned char> input;
        size_t size;

        // Read from file
        {
            std::scoped_lock lock {_mtx};

            if (!_input_file || !_input_file->is_open()) {
                throw std::runtime_error("No file to read");
            }

            if (_input_file->eof()) {
                return false;
            }

            _input_file->read(reinterpret_cast<char*>(&data_type), sizeof (DataType));

            if (data_type != DataType::LEFT && data_type != DataType::RIGHT && data_type != DataType::IMU) {
                return false;
            }

            _input_file->read(reinterpret_cast<char*>(&timestamp), sizeof (uint64_t));
            _input_file->read(reinterpret_cast<char*>(&size), sizeof (size_t));
            input.resize(size);
            _input_file->read(reinterpret_cast<char*>(input.data()), size * sizeof (unsigned char));
        }

        // Decode
        switch (data_type)
        {
            case DataType::LEFT:
                [[fallthrough]];
            case DataType::RIGHT:
                image = cv::imdecode(input, cv::IMREAD_GRAYSCALE);
                break;
            case DataType::IMU:
                throw std::runtime_error("NOT IMPLEMENTED");
        }

        return true;
    }

    bool _read_image_pair(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp)
    {
        while (_queue_left.empty() || _queue_right.empty() ||
                _queue_left.front().timestamp != _queue_right.front().timestamp)
        {
            cv::Mat frame;
            uint64_t timestamp;
            DataType data_type;
            if (!_read_datum(frame, timestamp, data_type)) {
                _has_frames = false;
                return false;
            }

            _enqueue_image(frame, timestamp, data_type);
            _discard_old_image();
        }

        _dequeue_image_pair(frame_left, frame_right, timestamp);
        return true;
    }

    bool _grab_image_pair(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp)
    {
        _api->WaitForStreams();

        _left_data = _api->GetStreamData(mynteye::Stream::LEFT_RECTIFIED);
        _right_data = _api->GetStreamData(mynteye::Stream::RIGHT_RECTIFIED);
        if (_left_data.frame.empty() || _right_data.frame.empty()) {
            return false;
        }

        if (_left_data.img->timestamp != _right_data.img->timestamp) {
            spdlog::warn("Frames out of sync at time {0} (left) and {1} (right)",
                    _left_data.img->timestamp,
                    _right_data.img->timestamp);
            return false;
        }

        timestamp = static_cast<double>(_left_data.img->timestamp) / 1e6;
        frame_left = _left_data.frame;
        frame_right = _right_data.frame;
        return true;
    }

    void _discard_old_image(void)
    {
        if (_queue_left.size() > 10 || _queue_right.size() > 10) {
            if (_queue_left.front().timestamp > _queue_right.front().timestamp) {
                _queue_left.pop();
            }
            else {
                _queue_right.pop();
            }
        }
    }
};

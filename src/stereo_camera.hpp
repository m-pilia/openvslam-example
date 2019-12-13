#pragma once

#include <yaml.h>
#include <opencv2/core/core.hpp>

class StereoCamera
{
public:
    explicit StereoCamera(
            const std::string& config_file_path,
            const std::string& input_file_path,
            const std::string& output_file_path
            )
        : _yaml_node {YAML::LoadFile(config_file_path)}
        , _config_file_path {config_file_path}
        , _input_file_path {input_file_path}
        , _output_file_path {output_file_path}
        , _playback {!_input_file_path.empty()}
        , _recording {!_output_file_path.empty()}
    {
    }

    virtual ~StereoCamera() {}

    virtual const YAML::Node& get_config(void)
    {
        return _yaml_node;
    }

    virtual void cleanup(void) = 0;
    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, double& timestamp) = 0;
    virtual bool has_frames(void) = 0;

protected:
    YAML::Node _yaml_node;
    const std::string _config_file_path;
    const std::string _input_file_path;
    const std::string _output_file_path;
    const bool _playback;
    const bool _recording;
};

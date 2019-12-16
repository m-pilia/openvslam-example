#pragma once

#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>

namespace CameraSlam {

class StereoCamera
{
public:
    explicit StereoCamera(
            const std::string& config_file_path,
            const std::string& input_file_path,
            const std::string& output_file_path
            );

    virtual ~StereoCamera() {}

    /*!
     * \brief Get an OpenVSLAM configuration object.
     */
    virtual const YAML::Node& get_config(void);

    /*!
     * \brief Clenup called on termination or error handling.
     */
    virtual void cleanup(void) = 0;

    /*!
     * \brief Grab a stereo image pair (from the camera or file playback).
     */
    virtual bool grab(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& rgb, cv::Mat& depth, double& timestamp) = 0;

    /*!
     * \brief Return `true` if more frames are available to read.
     */
    virtual bool has_frames(void) = 0;

protected:
    YAML::Node _yaml_node;
    const std::string _config_file_path;
    const std::string _input_file_path;
    const std::string _output_file_path;
    const bool _playback;
    const bool _recording;
    const bool _rgbd_mode;
};

} // namespace CameraSlam

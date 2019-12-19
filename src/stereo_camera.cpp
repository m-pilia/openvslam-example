#include "stereo_camera.hpp"

namespace CameraSlam {

StereoCamera::StereoCamera(
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
    , _mode {_get_mode()}
{
}

const YAML::Node& StereoCamera::get_config(void)
{
    return _yaml_node;
}

StereoCamera::Mode StereoCamera::_get_mode(void)
{
    if (_yaml_node["Camera.setup"].IsDefined() && _yaml_node["Camera.setup"].as<std::string>() == "RGBD") {
        return Mode::RGBD;
    }
    else if (_yaml_node["StereoRectifier.model"].IsDefined() && _yaml_node["StereoRectifier.model"].as<std::string>() == "fisheye") {
        return Mode::FISHEYE;
    }
    else {
        return Mode::PERSPECTIVE;
    }
}

} // namespace CameraSlam

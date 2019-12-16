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
{
}

const YAML::Node& StereoCamera::get_config(void)
{
    return _yaml_node;
}

} // namespace CameraSlam

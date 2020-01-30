[![](https://github.com/m-pilia/openvslam-example/workflows/Checks/badge.svg)](https://github.com/m-pilia/openvslam-example/actions?query=workflow%3AChecks)
[![](https://img.shields.io/docker/cloud/build/martinopilia/openvslam-example-env)](https://hub.docker.com/r/martinopilia/openvslam-example-env)

# Stereo camera SLAM

This repository contains a simple example of SLAM setup, using
[OpenVSLAM](https://github.com/xdspacelab/openvslam.git) with different stereo
cameras (ZED, RealSense, MYNT EYE).

# Caveat emptor
- This package is not meant as production-ready code, but rather as a simple
  example to test different cameras in real-time with OpenVSLAM. The
  implementation is not robust, testing is minimal, and many features have a
  stub or oversimplified implementation.
- The ZED module was not tested on hardware after refactoring and porting to
  version 3.0 of the ZED SDK, so it is possible that it does not work
  properly in its current state.

# Configuration

The `camera_slam` executable requires a configuration file in the OpenVSLAM
format. The main difference is that the configuration does not require to
contain the calibration parameters, that will be directly read from the
camera, and any calibration parameter the file will be ignored. Example
configurations are provided in the `param` folder.

When the camera supports multiple resolutions and frame rates, the settings
from the configuration file will be applied. The RealSense devices can be
used either in stereo mode (D435, setting frame rate and resolution in the
configuration file), RGBD mode (D435, setting frame rate, resolution, and
`Camera.setup: RGBD`), or stereo fisheye (T265 setting
`StereoRectifier.model: fisheye`). Please refer to the configuration files in
the `./param` folder for an example.

An ORB vocabulary is also required. A pre-built vocabulary [can be
downloaded](https://openvslam.readthedocs.io/en/master/simple_tutorial.html)
from the OpenVSLAM project.

# Native build

Requires [CMake](https://cmake.org/) >= 3.8 and [gcc](https://gcc.gnu.org/) >= 7.2.

Install the dependencies by following upstream instructions:
- [OpenVSLAM](https://openvslam.readthedocs.io/en/master/installation.html).
  Note that OpenVSLAM requires [OpenCV](https://github.com/opencv/opencv) >= 3.3,
  and MYNT-EYE-S-SDK requires OpenCV < 4.0.
- [MYNT-EYE-S-SDK](https://mynt-eye-s-sdk.readthedocs.io/en/latest/src/sdk/install_ubuntu_src.html)
  2.5 (build from source to avoid binary conflicts with OpenCV).
- [ZED SDK](https://www.stereolabs.com/developers/release) 3.0 (optional, only for ZED camera, requires CUDA).
- [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) (recommended: 2.32.0).

Build:
```
git submodule update --init --recursive
mkdir build && cd build
cmake .. -DCAMERA_SLAM_WITH_ZED=OFF # change to ON to build with support for the ZED camera
make
```

Run:
```
./camera_slam --help
```

# Point cloud visualisation

A simple Python script (located in the `./scripts` folder) demonstrates how
to load the map file, saved by OpenVSLAM as `.msg`, and visualise the point
cloud or export it to CSV.

Install QML, if not already present, and set-up a virtual
environment with the Python dependencies (example for Ubuntu 18.04)
```
apt-get update -y -qq
apt-get install -y -qq --no-install-recommends \
        python3 \
        python3-pip \
        python3-pyqt5 \
        python3-pyqt5.qtopengl \
        python3-pyqt5.qtquick \
        python3-setuptools \
        python3-venv \
        qml-module-* \
        qmlscene

python3 -m venv ./scripts/.venv
source ./scripts/.venv/bin/activate
pip install -r ./scripts/requirements.txt
```
then run
```
python ./scripts/read_map.py
```

# Container build

A Dockerfile is provided to build the project without ZED support (leaner
image, no CUDA-capable GPU required on the host). A second Dockerfile allows
to build with ZED support.

Note that the Docker images are provided mainly *for testing*, and the
application may not operate properly within the containers. Moreover, running
an OpenGL application within a Docker container may fail due to a mismatching
between the graphics driver on the host and in the container.

## Without ZED

Get the Docker image
```
image=martinopilia/openvslam-example-env:latest
docker pull ${image} || docker build --rm -t ${image} -f Dockerfile .
```
then build
```
git submodule update --init --recursive
docker run \
        --rm \
        -v "$(pwd)":/mnt \
        ${image} \
        bash -c '\
            set -xeuo pipefail && \
            mkdir -p /mnt/build && \
            cd /mnt/build && \
            cmake .. && \
            make -j"$(nproc)" \
        '
```
and run
```
docker run \
        --rm \
        --net=host \
        --privileged \
        -v "$(pwd)":/mnt \
        ${image} \
        /mnt/build/camera_slam --help
```

## With ZED

To build with ZED support (larger Docker image, requires the [NVIDIA
Container Toolkit](https://github.com/NVIDIA/nvidia-docker)), build the ZED
Docker image
```
image=martinopilia/openvslam-example-env:latest-zed
docker pull ${image} || docker build --rm -t ${image} -f Dockerfile.zed .
```
then build
```
git submodule update --init --recursive
docker run \
        --rm \
        --gpus all \
        -v "$(pwd)":/mnt \
        ${image} \
        bash -c '\
            set -xeuo pipefail && \
            mkdir -p /mnt/build && \
            cd /mnt/build && \
            cmake .. -DCAMERA_SLAM_WITH_ZED=ON && \
            make -j"$(nproc)" \
        '
```
and run
```
docker run \
        --rm \
        --net=host \
        --privileged \
        --gpus all \
        -v "$(pwd)":/mnt \
        ${image} \
        /mnt/build/camera_slam --help
```

# License

MIT
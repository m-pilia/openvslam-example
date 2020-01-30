# hadolint ignore=DL3007
FROM shinsumicco/openvslam-env:latest

LABEL maintainer="Martino Pilia <martino.pilia@gmail.com>"

SHELL ["/bin/bash", "-x", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND noninteractive

# hadolint ignore=DL3008
RUN apt-get update -y -qq \
&&  apt-get install -y -qq --no-install-recommends \
        apt-transport-https \
        ca-certificates \
        software-properties-common \
&&  rm -rf /var/lib/apt/lists/*

# Update CMake (to get C++17 support)
# hadolint ignore=DL3008
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - \
&&  apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main' \
&&  apt-get update -qq \
&&  apt-get install -y -qq --no-install-recommends cmake \
&&  rm -rf /var/lib/apt/lists/*

# Update gcc (to get full C++17 support)
# hadolint ignore=DL3008
RUN add-apt-repository ppa:ubuntu-toolchain-r/test \
&&  apt-get update -qq \
&&  apt-get install -y -qq --no-install-recommends g++-7 \
&&  rm -rf /var/lib/apt/lists/* \
&&  update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-7 \
&&  update-alternatives --config gcc

# Install librealsense
# hadolint ignore=DL3008
RUN ( \
        apt-key adv \
            --keyserver keys.gnupg.net \
            --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    ||  apt-key adv \
            --keyserver hkp://keyserver.ubuntu.com:80 \
            --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    ) \
&&  add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u \
&&  apt-get update -y -qq \
&&  apt-get install -y -qq --no-install-recommends \
        librealsense2-dev \
        librealsense2-dbg \
&&  apt-get autoremove -y -qq \
&&  rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

# Install MYNT EYE S SDK
# hadolint ignore=DL3003,DL3008
RUN apt-get update -y -qq \
&&  apt-get install -y -qq --no-install-recommends \
        libboost-filesystem-dev \
&&  apt-get autoremove -y -qq \
&&  rm -rf /var/lib/apt/lists/* \
&&  git clone https://github.com/slightech/MYNT-EYE-S-SDK.git mynt_eye_s_sdk \
&&  mkdir -p /tmp/mynt_eye_s_sdk/build \
&&  cd /tmp/mynt_eye_s_sdk/build \
&&  cmake .. \
&&  make -j"$(nproc)" \
&&  make install \
&&  rm -rf ./*

# Build OpenVSLAM
ARG OPENVSLAM_COMMIT=7f1b2bae2e938ee930e31316db44ed47a300095d
# hadolint ignore=DL3003
RUN git clone https://github.com/xdspacelab/openvslam.git /tmp/openvslam \
&&  mkdir -p /tmp/openvslam/build \
&&  cd /tmp/openvslam/build \
&&  git checkout $OPENVSLAM_COMMIT \
&&  cmake \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DBUILD_WITH_MARCH_NATIVE=OFF \
        -DUSE_PANGOLIN_VIEWER=ON \
        -DUSE_SOCKET_PUBLISHER=OFF \
        -DBOW_FRAMEWORK=DBoW2 \
        -DBUILD_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DINSTALL_PANGOLIN_VIEWER=ON \
        .. \
&&  make -j"$(nproc)" \
&&  make install \
&&  rm -rf ./*

# Cleanup
RUN rm -rf /tmp/*

WORKDIR /

cmake_minimum_required(VERSION 3.1)
project(slam LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_NO_SYSTEM_FROM_IMPORTED ON)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_definitions(-DUSE_PANGOLIN_VIEWER)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math")

include_directories(${PROJECT_SOURCE_DIR}/src)

find_package(openvslam REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(PNG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# OpenCV 3 for compatibility with MyntEye
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})

find_package(spdlog REQUIRED)
include_directories(${spdlog_INCLUDE_DIR})

find_package(ZED 3 REQUIRED)
include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})

find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

find_package(realsense2 REQUIRED )
include_directories(include ${realsense_INCLUDE_DIR})

find_package(mynteye REQUIRED)
message(STATUS "Found mynteye: ${mynteye_VERSION}")

find_package(Boost REQUIRED COMPONENTS filesystem) # Required my MyntEye

if(USE_BACKWARD)
    find_package(Backward REQUIRED)
    add_definitions(-DUSE_BACKWARD)
endif()

include_directories(third_party/popl/include)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

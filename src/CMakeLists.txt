set(EXECUTABLE_TARGETS "")

add_executable(camera_slam camera_slam.cc)
list(APPEND EXECUTABLE_TARGETS camera_slam)

target_sources(camera_slam PRIVATE
        mynteye_camera.cpp
        mynteye_camera.hpp
        realsense_camera.cpp
        realsense_camera.hpp
        stack_trace.cpp
        stack_trace.hpp
        stereo_camera.cpp
        stereo_camera.hpp
        stereo_slam.hpp
        zed_camera.cpp
        zed_camera.hpp
        )

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
            )

    target_link_libraries(${EXECUTABLE_TARGET} PUBLIC
        m
        openvslam::openvslam
        openvslam::pangolin_viewer
        Eigen3::Eigen
        yaml-cpp
        opencv_calib3d
        opencv_core
        opencv_features2d
        opencv_highgui
        opencv_imgcodecs
        ${GLEW_LIBRARIES}
        ${ZED_LIBS}
        ${realsense2_LIBRARY}
        mynteye
        Boost::filesystem
        ${OPENGL_gl_LIBRARY}
        )

    if(USE_BACKWARD)
        add_backward(${EXECUTABLE_TARGET})
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${CMAKE_DL_LIBS})
    endif()
endforeach()

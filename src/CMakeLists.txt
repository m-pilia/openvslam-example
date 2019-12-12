set(EXECUTABLE_TARGETS "")

add_executable(camera_slam camera_slam.cc)
list(APPEND EXECUTABLE_TARGETS camera_slam)

target_sources(camera_slam PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/stereo_camera.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/zed_camera.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/realsense_camera.hpp
        )

add_executable(video_slam video_slam.cc)
list(APPEND EXECUTABLE_TARGETS video_slam)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
            )

        target_link_libraries(${EXECUTABLE_TARGET} PUBLIC
        openvslam::openvslam
        openvslam::pangolin_viewer
        Eigen3::Eigen
        yaml-cpp
        opencv_core
        ${GLEW_LIBRARIES}
        ${OpenCV_LIBS}
        ${ZED_LIBS}
        ${realsense2_LIBRARY}
        )

    if(USE_BACKWARD)
        add_backward(${EXECUTABLE_TARGET})
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${CMAKE_DL_LIBS})
    endif()
endforeach()

cmake_minimum_required(VERSION 3.16)

project(slam_camera_contracts VERSION 0.1.0 LANGUAGES CXX)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

option(SLAM_BUILD_TESTS "Build SLAM adapter contract tests" ON)
option(SLAM_ENABLE_ORB_SLAM3 "Build the optional ORB-SLAM3 adapter" OFF)
set(SLAM_ORB_SLAM3_ROOT "" CACHE PATH "Path to a patched ORB-SLAM3 source tree")
if(SLAM_BUILD_TESTS)
    enable_testing()
endif()

add_library(slam_camera_contracts INTERFACE)
add_library(slam::camera_contracts ALIAS slam_camera_contracts)
target_compile_features(slam_camera_contracts INTERFACE cxx_std_17)
target_include_directories(
    slam_camera_contracts
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_library(slam_recorded_frame_source src/recorded_frame_source.cpp)
add_library(slam::recorded_frame_source ALIAS slam_recorded_frame_source)
target_link_libraries(slam_recorded_frame_source PUBLIC slam::camera_contracts)

add_library(slam_calibration_io src/calibration_io.cpp)
add_library(slam::calibration_io ALIAS slam_calibration_io)
target_link_libraries(slam_calibration_io PUBLIC slam::camera_contracts)

add_library(slam_point_cloud_delta_reducer src/point_cloud_delta_reducer.cpp)
add_library(slam::point_cloud_delta_reducer ALIAS slam_point_cloud_delta_reducer)
target_link_libraries(slam_point_cloud_delta_reducer PUBLIC slam::camera_contracts)

if(SLAM_ENABLE_ORB_SLAM3)
    if(NOT IS_DIRECTORY "${SLAM_ORB_SLAM3_ROOT}")
        message(FATAL_ERROR "SLAM_ORB_SLAM3_ROOT must name an ORB-SLAM3 source tree")
    endif()
    find_package(OpenCV 4 REQUIRED COMPONENTS core)
    find_package(Eigen3 3.3 REQUIRED NO_MODULE)
    find_package(Boost 1.70 REQUIRED COMPONENTS serialization)
    find_package(Pangolin REQUIRED)
    find_package(GLEW REQUIRED)
    add_library(
        slam_orbslam3_adapter
        src/orbslam3_tracked_map_points.cpp
        src/orbslam3_monocular_tracker.cpp
    )
    add_library(slam::orbslam3_adapter ALIAS slam_orbslam3_adapter)
    target_include_directories(
        slam_orbslam3_adapter
        SYSTEM PRIVATE
            "${SLAM_ORB_SLAM3_ROOT}"
            "${SLAM_ORB_SLAM3_ROOT}/include"
            "${SLAM_ORB_SLAM3_ROOT}/include/CameraModels"
            "${SLAM_ORB_SLAM3_ROOT}/Thirdparty/Sophus"
            "${SLAM_ORB_SLAM3_ROOT}/Thirdparty/g2o/compat"
    )
    target_link_libraries(
        slam_orbslam3_adapter
        PUBLIC slam::point_cloud_delta_reducer
        PRIVATE
            Eigen3::Eigen
            Boost::serialization
            ${Pangolin_LIBRARIES}
            GLEW::GLEW
            opencv_core
            "${SLAM_ORB_SLAM3_ROOT}/lib/libORB_SLAM3.dylib"
    )
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(slam_orbslam3_adapter PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()

endif()

add_executable(recorded_frame_dump tools/recorded_frame_dump.cpp)
target_link_libraries(recorded_frame_dump PRIVATE slam::recorded_frame_source)

add_executable(calibration_convert tools/calibration_convert.cpp)
target_link_libraries(calibration_convert PRIVATE slam::calibration_io)

if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
    target_compile_options(slam_recorded_frame_source PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(recorded_frame_dump PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(slam_calibration_io PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(calibration_convert PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(
        slam_point_cloud_delta_reducer PRIVATE -Wall -Wextra -Wpedantic -Werror
    )
endif()

if(APPLE)
    enable_language(OBJCXX)
    add_library(slam_macos_camera_source src/macos_camera_source.mm)
    add_library(slam::macos_camera_source ALIAS slam_macos_camera_source)
    target_link_libraries(
        slam_macos_camera_source
        PUBLIC slam::camera_contracts
        PRIVATE
            "-framework AVFoundation"
            "-framework CoreMedia"
            "-framework CoreVideo"
            "-framework Foundation"
    )
    target_compile_options(
        slam_macos_camera_source
        PRIVATE -fobjc-arc -Wall -Wextra -Wpedantic -Werror
    )

    add_executable(macos_camera_dump tools/macos_camera_dump.cpp)
    target_link_libraries(macos_camera_dump PRIVATE slam::macos_camera_source)
    target_compile_options(macos_camera_dump PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_options(
        macos_camera_dump
        PRIVATE
            "SHELL:-Wl,-sectcreate,__TEXT,__info_plist,${CMAKE_CURRENT_SOURCE_DIR}/macos/CameraInfo.plist"
    )

    if(SLAM_ENABLE_ORB_SLAM3)
        add_executable(macos_orbslam3_live tools/macos_orbslam3_live.cpp)
        target_link_libraries(
            macos_orbslam3_live
            PRIVATE
                slam::macos_camera_source
                slam::calibration_io
                slam::orbslam3_adapter
                slam::point_cloud_delta_reducer
        )
        target_compile_options(
            macos_orbslam3_live PRIVATE -Wall -Wextra -Wpedantic -Werror
        )
        target_link_options(
            macos_orbslam3_live
            PRIVATE
                "SHELL:-Wl,-sectcreate,__TEXT,__info_plist,${CMAKE_CURRENT_SOURCE_DIR}/macos/CameraInfo.plist"
        )
    endif()

    if(SLAM_BUILD_TESTS)
        add_executable(macos_camera_source_tests tests/macos_camera_source_tests.cpp)
        target_link_libraries(macos_camera_source_tests PRIVATE slam::macos_camera_source)
        target_compile_options(macos_camera_source_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        add_test(NAME macos_camera_source_tests COMMAND macos_camera_source_tests)
    endif()
endif()

if(SLAM_BUILD_TESTS)
    add_executable(camera_contract_tests tests/camera_contract_tests.cpp)
    target_link_libraries(camera_contract_tests PRIVATE slam::camera_contracts)
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(camera_contract_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME camera_contract_tests COMMAND camera_contract_tests)

    add_executable(recorded_frame_source_tests tests/recorded_frame_source_tests.cpp)
    target_link_libraries(recorded_frame_source_tests PRIVATE slam::recorded_frame_source)
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(recorded_frame_source_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME recorded_frame_source_tests COMMAND recorded_frame_source_tests)

    add_executable(live_frame_queue_tests tests/live_frame_queue_tests.cpp)
    target_link_libraries(live_frame_queue_tests PRIVATE slam::camera_contracts)
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(live_frame_queue_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME live_frame_queue_tests COMMAND live_frame_queue_tests)

    add_executable(calibration_io_tests tests/calibration_io_tests.cpp)
    target_link_libraries(calibration_io_tests PRIVATE slam::calibration_io)
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(calibration_io_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME calibration_io_tests COMMAND calibration_io_tests)

    add_executable(point_cloud_delta_reducer_tests tests/point_cloud_delta_reducer_tests.cpp)
    target_link_libraries(
        point_cloud_delta_reducer_tests PRIVATE slam::point_cloud_delta_reducer
    )
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(
            point_cloud_delta_reducer_tests PRIVATE -Wall -Wextra -Wpedantic -Werror
        )
    endif()
    add_test(NAME point_cloud_delta_reducer_tests COMMAND point_cloud_delta_reducer_tests)
endif()

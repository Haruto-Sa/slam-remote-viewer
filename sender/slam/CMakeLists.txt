cmake_minimum_required(VERSION 3.16)

project(slam_camera_contracts VERSION 0.1.0 LANGUAGES CXX)

option(SLAM_BUILD_TESTS "Build SLAM adapter contract tests" ON)
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

add_executable(recorded_frame_dump tools/recorded_frame_dump.cpp)
target_link_libraries(recorded_frame_dump PRIVATE slam::recorded_frame_source)

add_executable(calibration_convert tools/calibration_convert.cpp)
target_link_libraries(calibration_convert PRIVATE slam::calibration_io)

if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
    target_compile_options(slam_recorded_frame_source PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(recorded_frame_dump PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(slam_calibration_io PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(calibration_convert PRIVATE -Wall -Wextra -Wpedantic -Werror)
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
endif()

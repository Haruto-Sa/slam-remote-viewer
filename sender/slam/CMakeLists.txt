cmake_minimum_required(VERSION 3.16)

project(slam_camera_contracts VERSION 0.1.0 LANGUAGES CXX)

option(SLAM_BUILD_TESTS "Build SLAM adapter contract tests" ON)

add_library(slam_camera_contracts INTERFACE)
add_library(slam::camera_contracts ALIAS slam_camera_contracts)
target_compile_features(slam_camera_contracts INTERFACE cxx_std_17)
target_include_directories(
    slam_camera_contracts
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(SLAM_BUILD_TESTS)
    enable_testing()
    add_executable(camera_contract_tests tests/camera_contract_tests.cpp)
    target_link_libraries(camera_contract_tests PRIVATE slam::camera_contracts)
    if(CMAKE_CXX_COMPILER_ID MATCHES "AppleClang|Clang|GNU")
        target_compile_options(camera_contract_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME camera_contract_tests COMMAND camera_contract_tests)
endif()

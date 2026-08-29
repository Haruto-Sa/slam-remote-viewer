#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "slam_remote/camera/calibration_io.hpp"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: calibration_convert INPUT.calibration OUTPUT.yaml\n";
        return EXIT_FAILURE;
    }
    try {
        const auto document =
            slam_remote::camera::LoadCalibrationDocument(std::filesystem::path(argv[1]));
        const auto yaml = slam_remote::camera::ToOrbSlam3MonocularYaml(document);
        std::ofstream output(argv[2]);
        if (!output) {
            throw std::runtime_error("cannot open output file");
        }
        output << yaml;
        if (!output) {
            throw std::runtime_error("failed to write output file");
        }
        std::cout << "validated device_id=" << document.camera.device_id
                  << " mode=" << document.camera.width << 'x' << document.camera.height << '@'
                  << document.fps << " rms_px=" << document.rms_reprojection_error_px << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "calibration conversion failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

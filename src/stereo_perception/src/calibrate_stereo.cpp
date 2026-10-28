#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "stereo_perception/stereo_pipeline.hpp"
int main(int argc, char** argv) {
  if (argc != 9) {
    std::cerr
        << "usage: calibrate_stereo LEFT_GLOB RIGHT_GLOB COLUMNS ROWS SQUARE_METERS OUTPUT.yaml LEFT_CAMERA_ID RIGHT_CAMERA_ID\n";
    return 2;
  }
  try {
    std::vector<cv::String> left_paths, right_paths;
    cv::glob(argv[1], left_paths);
    cv::glob(argv[2], right_paths);
    if (left_paths.size() != right_paths.size() || left_paths.size() < 8)
      throw std::runtime_error("at least eight corresponding stereo pairs required");
    std::vector<cv::Mat> left, right;
    for (std::size_t i = 0; i < left_paths.size(); ++i) {
      left.push_back(cv::imread(left_paths[i], cv::IMREAD_GRAYSCALE));
      right.push_back(cv::imread(right_paths[i], cv::IMREAD_GRAYSCALE));
    }
    double usable{};
    auto calibration = robotics::perception::calibrate_chessboard(
        left, right, {std::stoi(argv[3]), std::stoi(argv[4])}, std::stod(argv[5]), &usable);
    calibration.left_camera_id = argv[7];
    calibration.right_camera_id = argv[8];
    if (calibration.left_camera_id.empty() || calibration.right_camera_id.empty() ||
        calibration.left_camera_id == calibration.right_camera_id ||
        calibration.left_camera_id.find("UNCONFIGURED") != std::string::npos ||
        calibration.right_camera_id.find("UNCONFIGURED") != std::string::npos)
      throw std::invalid_argument("distinct measured left/right camera identities are required");
    if (calibration.rms_error > 1.0 || calibration.left_rms_error > 1.0 ||
        calibration.right_rms_error > 1.0 || usable * left.size() < 8)
      throw std::runtime_error(
          "calibration quality rejected: require each RMS <= 1 pixel and 8 detected pairs");
    calibration.save(argv[6]);
    std::cout << "stereo_rms_pixels=" << calibration.rms_error
              << " left_rms_pixels=" << calibration.left_rms_error
              << " right_rms_pixels=" << calibration.right_rms_error
              << " baseline_m=" << calibration.baseline()
              << " usable_fraction=" << usable << '\n';
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

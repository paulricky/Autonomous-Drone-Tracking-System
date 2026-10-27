#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "ece3162_perception/stereo_pipeline.hpp"
int main(int argc, char** argv) {
  if (argc != 7) {
    std::cerr
        << "usage: calibrate_stereo LEFT_GLOB RIGHT_GLOB COLUMNS ROWS SQUARE_METERS OUTPUT.yaml\n";
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
    auto calibration = ece3162::perception::calibrate_chessboard(
        left, right, {std::stoi(argv[3]), std::stoi(argv[4])}, std::stod(argv[5]), &usable);
    if (calibration.rms_error > 1.0 || usable * left.size() < 8)
      throw std::runtime_error(
          "calibration quality rejected: require RMS <= 1 pixel and 8 detected pairs");
    calibration.save(argv[6]);
    std::cout << "rms_pixels=" << calibration.rms_error << " baseline_m=" << calibration.baseline()
              << " usable_fraction=" << usable << '\n';
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

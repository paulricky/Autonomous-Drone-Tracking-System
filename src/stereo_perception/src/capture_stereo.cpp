#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

#include "stereo_perception/stereo_pipeline.hpp"

int main(int argc, char** argv) {
  if (argc != 9 && argc != 10) {
    std::cerr << "usage: capture_stereo LEFT_DEVICE RIGHT_DEVICE WIDTH HEIGHT FPS MAX_DELTA_MS COUNT OUTPUT_DIR [FOURCC]\n";
    return 2;
  }
  try {
    namespace fs = std::filesystem;
    const std::string left = argv[1], right = argv[2];
    const int width = std::stoi(argv[3]), height = std::stoi(argv[4]), count = std::stoi(argv[7]);
    const double fps = std::stod(argv[5]), tolerance = std::stod(argv[6]) / 1000.;
    if (left == right || left.find("UNCONFIGURED") != std::string::npos ||
        right.find("UNCONFIGURED") != std::string::npos ||
        !fs::exists(left) || !fs::exists(right) || fs::equivalent(left, right) ||
        (left.rfind("/dev/v4l/by-id/", 0) != 0 && left.rfind("/dev/v4l/by-path/", 0) != 0) ||
        (right.rfind("/dev/v4l/by-id/", 0) != 0 && right.rfind("/dev/v4l/by-path/", 0) != 0) ||
        width < 16 || height < 16 ||
        !std::isfinite(fps) || fps <= 0 || fps > 240 || !std::isfinite(tolerance) ||
        tolerance <= 0 || count < 8 || count > 10000)
      throw std::invalid_argument("invalid capture devices or settings");
    const fs::path output(argv[8]);
    if (fs::exists(output) && !fs::is_empty(output))
      throw std::invalid_argument("capture output must be new or empty; existing measurements are preserved");
    fs::create_directories(output);
    int fourcc = 0;
    if (argc == 10) {
      const std::string format = argv[9];
      if (format.size() != 4)
        throw std::invalid_argument("FOURCC must have exactly four characters");
      fourcc = cv::VideoWriter::fourcc(format[0], format[1], format[2], format[3]);
    }
    robotics::perception::UvcCamera l({left, "left", width, height, fps, -1, -1, fourcc});
    robotics::perception::UvcCamera r({right, "right", width, height, fps, -1, -1, fourcc});
    std::ofstream manifest(output / "manifest.csv");
    if (!manifest)
      throw std::runtime_error("cannot write capture manifest");
    manifest << "pair,left_device,right_device,left_monotonic_s,right_monotonic_s,delta_s\n";
    int accepted = 0;
    for (; accepted < count; ++accepted) {
      std::cout << "Position the stationary checkerboard at a new pose, then press Enter for pair "
                << accepted << '/' << count << std::endl;
      std::string input;
      if (!std::getline(std::cin, input)) break;
      bool saved = false;
      for (int attempt = 0; attempt < 20; ++attempt) {
        // Independent USB cameras must be acquired concurrently. Sequential blocking
        // reads introduce a full frame period of artificial timestamp skew.
        auto left_read = std::async(std::launch::async, [&l] { return l.read(); });
        const auto b = r.read();
        const auto a = left_read.get();
        if (!a || !b || std::abs(a->receive_time - b->receive_time) > tolerance)
          continue;
        const auto number = std::to_string(accepted);
        if (!cv::imwrite((output / ("left_" + number + ".png")).string(), a->image) ||
            !cv::imwrite((output / ("right_" + number + ".png")).string(), b->image))
          throw std::runtime_error("cannot write calibration images");
        manifest << accepted << ',' << left << ',' << right << ',' << std::setprecision(17)
                 << a->receive_time << ',' << b->receive_time << ','
                 << std::abs(a->receive_time - b->receive_time) << '\n';
        saved = true;
        break;
      }
      if (!saved) break;
    }
    std::cout << "accepted_pairs=" << accepted << " requested_pairs=" << count << '\n';
    return accepted == count ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

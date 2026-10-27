#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>

#include "ece3162_core/voxel_map.hpp"
#include "ece3162_perception/state_estimator.hpp"
#include "ece3162_perception/stereo_pipeline.hpp"
#include "ece3162_perception/stereo_synchronizer.hpp"
namespace ep = ece3162::perception;
namespace nav = ece3162::navigation;
using Clock = std::chrono::steady_clock;
static double ms(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
int main(int argc, char** argv) {
  cv::setNumThreads(1);
  const std::string directory = argc > 1 ? argv[1] : "results/perception/components";
  std::filesystem::create_directories(directory);
  std::ofstream out(directory + "/components.csv");
  out << std::setprecision(12)
      << "frame,render_ms,sync_ms,rectify_ms,disparity_ms,depth_ms,cloud_ms,map_ms,vo_ms,estimator_"
         "ms,total_ms,valid_fraction,disparity_mae_px,depth_mae_m,depth_relative_error,cloud_rmse_"
         "m,vo_position_error_m,map_precision,map_recall,map_cells,vo_valid\n";
  ep::SyntheticConfig cfg;
  cfg.baseline = .16;
  ep::SyntheticStereo renderer(cfg);
  ep::StereoConfig stereo_cfg;
  stereo_cfg.num_disparities = 64;
  ep::StereoPipeline pipeline(renderer.calibration(), stereo_cfg);
  ep::StereoVisualOdometry vo(cfg.fx, cfg.fy, cfg.cx, cfg.cy);
  ep::StateEstimator estimator;
  ep::StereoSynchronizer sync(.005, 4);
  nav::VoxelMap map({-1, -2, 0}, {7, 2, 3}, .25), reference = map;
  std::vector<ep::SceneBox> boxes = {{{2.5, -.4, 0}, {2.9, .4, 2}}};
  for (int i = 0; i < 35; ++i) {
    double stamp = 1 + i * .1;
    auto pose = ep::camera_pose({i * .01, 0, 1});
    auto begin = Clock::now();
    auto frame = renderer.render_pose(stamp, pose, boxes);
    double render_ms = ms(begin);
    auto st = Clock::now();
    sync.push_left({std::uint64_t(i), stamp});
    sync.push_right({std::uint64_t(i), stamp});
    if (!sync.pop_pair(stamp, .25))
      return 2;
    double sync_ms = ms(st);
    auto depth = pipeline.process(frame.left, frame.right);
    double cloud_ms{};
    auto points = pipeline.point_cloud(depth, 4, &cloud_ms);
    auto result = vo.update(depth.rectified_left, depth.depth, stamp);
    st = Clock::now();
    auto state = estimator.update(result, stamp);
    double estimate_ms = ms(st);
    std::vector<nav::Vec3> world, truth;
    double de = 0, ze = 0, relative = 0, cloud_squared = 0;
    std::size_t valid = 0, cloud_count = 0;
    for (int y = 0; y < depth.depth.rows; ++y)
      for (int x = 0; x < depth.depth.cols; ++x) {
        float z = depth.depth.at<float>(y, x), gt = frame.ground_truth_depth.at<float>(y, x);
        if (!std::isfinite(z))
          continue;
        ++valid;
        ze += std::abs(z - gt);
        relative += std::abs(z - gt) / gt;
        de += std::abs(depth.disparity.at<float>(y, x) - cfg.fx * cfg.baseline / gt);
        if (x % 4 == 0 && y % 4 == 0) {
          cv::Vec4d p((x - cfg.cx) * z / cfg.fx, (y - cfg.cy) * z / cfg.fy, z, 1),
              q((x - cfg.cx) * gt / cfg.fx, (y - cfg.cy) * gt / cfg.fy, gt, 1);
          double error = cv::norm(p - q);
          cloud_squared += error * error;
          ++cloud_count;
          auto w = pose * p, t = pose * q;
          world.push_back({w[0], w[1], w[2]});
          truth.push_back({t[0], t[1], t[2]});
        }
      }
    st = Clock::now();
    map.integrate_cloud({pose(0, 3), pose(1, 3), pose(2, 3)}, world, 8);
    double map_ms = ms(st);
    reference.integrate_cloud({pose(0, 3), pose(1, 3), pose(2, 3)}, truth, 8);
    auto occupied = map.occupied_centers(), actual = reference.occupied_centers();
    std::size_t tp = 0, found = 0;
    for (auto& p : occupied)
      if (reference.occupied(p, false))
        ++tp;
    for (auto& p : actual)
      if (map.occupied(p, false))
        ++found;
    cv::Vec3d error(state.world_from_camera(0, 3) - pose(0, 3),
                    state.world_from_camera(1, 3) - pose(1, 3),
                    state.world_from_camera(2, 3) - pose(2, 3));
    out << i << ',' << render_ms << ',' << sync_ms << ',' << depth.rectify_ms << ','
        << depth.disparity_ms << ',' << depth.depth_ms << ',' << cloud_ms << ',' << map_ms << ','
        << result.processing_ms << ',' << estimate_ms << ',' << ms(begin) << ','
        << double(valid) / depth.depth.total() << ',' << de / valid << ',' << ze / valid << ','
        << relative / valid << ',' << std::sqrt(cloud_squared / cloud_count) << ','
        << cv::norm(error) << ',' << double(tp) / std::max<std::size_t>(1, occupied.size()) << ','
        << double(found) / std::max<std::size_t>(1, actual.size()) << ','
        << map.shape()[0] * map.shape()[1] * map.shape()[2] << ',' << result.valid << '\n';
    if (i == 0) {
      renderer.calibration().save(directory + "/synthetic_calibration.yaml");
      cv::imwrite(directory + "/left.png", frame.left);
      cv::imwrite(directory + "/right.png", frame.right);
      cv::Mat disp;
      depth.disparity.convertTo(disp, CV_8U, 4);
      cv::imwrite(directory + "/disparity.png", disp);
    }
  }
  return 0;
}

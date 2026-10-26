#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace ece3162::perception {
struct FrameStamp { std::uint64_t sequence{}; double host_timestamp_seconds{}; };
struct StereoPair { FrameStamp left; FrameStamp right; double timestamp_delta_seconds{}; };
struct SynchronizerStats {
  std::size_t paired{}; std::size_t rejected{}; std::size_t queue_overflow{};
  std::size_t non_monotonic{};
};
class StereoSynchronizer {
 public:
  StereoSynchronizer(double max_delta_seconds, std::size_t max_queue);
  bool push_left(FrameStamp frame);
  bool push_right(FrameStamp frame);
  std::optional<StereoPair> pop_pair();
  const SynchronizerStats& stats() const noexcept { return stats_; }
 private:
  bool push(std::deque<FrameStamp>& queue, FrameStamp frame, std::optional<double>& latest);
  double max_delta_; std::size_t max_queue_; std::deque<FrameStamp> left_, right_;
  std::optional<double> latest_left_, latest_right_; SynchronizerStats stats_;
};
}  // namespace ece3162::perception

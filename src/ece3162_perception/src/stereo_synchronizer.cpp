#include "ece3162_perception/stereo_synchronizer.hpp"
#include <cmath>
#include <stdexcept>

namespace ece3162::perception {
StereoSynchronizer::StereoSynchronizer(double max_delta_seconds, std::size_t max_queue)
    : max_delta_(max_delta_seconds), max_queue_(max_queue) {
  if (!std::isfinite(max_delta_) || max_delta_ < 0.0 || max_queue_ == 0)
    throw std::invalid_argument("invalid stereo synchronization limits");
}
bool StereoSynchronizer::push(std::deque<FrameStamp>& queue, FrameStamp frame,
                              std::optional<double>& latest) {
  if (!std::isfinite(frame.host_timestamp_seconds) ||
      (latest && frame.host_timestamp_seconds < *latest)) {
    ++stats_.non_monotonic; return false;
  }
  latest = frame.host_timestamp_seconds;
  if (queue.size() == max_queue_) { queue.pop_front(); ++stats_.queue_overflow; }
  queue.push_back(frame); return true;
}
bool StereoSynchronizer::push_left(FrameStamp frame) { return push(left_, frame, latest_left_); }
bool StereoSynchronizer::push_right(FrameStamp frame) { return push(right_, frame, latest_right_); }
std::optional<StereoPair> StereoSynchronizer::pop_pair() {
  while (!left_.empty() && !right_.empty()) {
    const double delta = left_.front().host_timestamp_seconds - right_.front().host_timestamp_seconds;
    if (std::abs(delta) <= max_delta_) {
      StereoPair pair{left_.front(), right_.front(), std::abs(delta)};
      left_.pop_front(); right_.pop_front(); ++stats_.paired; return pair;
    }
    if (delta < 0.0) left_.pop_front(); else right_.pop_front();
    ++stats_.rejected;
  }
  return std::nullopt;
}
}  // namespace ece3162::perception

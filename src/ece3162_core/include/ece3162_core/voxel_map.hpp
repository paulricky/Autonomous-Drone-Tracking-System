#pragma once
#include "ece3162_core/navigation_types.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ece3162::navigation {
enum class Occupancy : std::uint8_t { kUnknown=0, kFree=1, kOccupied=2 };
class VoxelMap {
 public:
  VoxelMap(Vec3 minimum, Vec3 maximum, double resolution, Occupancy initial=Occupancy::kUnknown);
  bool contains(const Vec3& point) const noexcept;
  bool occupied(const Vec3& point, bool unknown_is_occupied=true) const noexcept;
  void set(const Vec3& point, Occupancy value);
  void fill_box(const Vec3& lower, const Vec3& upper, Occupancy value=Occupancy::kOccupied);
  void fill(Occupancy value) noexcept;
  bool segment_occupied(const Vec3& a, const Vec3& b, double spacing,
                        bool unknown_is_occupied=true, std::size_t* checks=nullptr) const;
  std::vector<Vec3> occupied_centers() const;
  const Vec3& minimum() const noexcept { return minimum_; }
  const Vec3& maximum() const noexcept { return maximum_; }
  double resolution() const noexcept { return resolution_; }
  const std::array<std::size_t,3>& shape() const noexcept { return shape_; }
 private:
  bool index(const Vec3&, std::array<std::size_t,3>&) const noexcept;
  std::size_t flat(const std::array<std::size_t,3>& i) const noexcept;
  Vec3 minimum_, maximum_; double resolution_; std::array<std::size_t,3> shape_{};
  std::vector<Occupancy> cells_;
};
}  // namespace ece3162::navigation

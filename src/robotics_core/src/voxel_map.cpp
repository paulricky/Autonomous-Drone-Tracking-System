#include "robotics_core/voxel_map.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotics::navigation {
void VoxelMap::integrate_cloud(const Vec3& origin, const std::vector<Vec3>& points, double range) {
  if (!finite(origin) || !std::isfinite(range) || range <= 0)
    throw std::invalid_argument("invalid occupancy cloud");
  std::vector<unsigned char> evidence(cells_.size(), 0);
  for (const auto& p : points) {
    if (!finite(p))
      continue;
    Vec3 delta = sub(p, origin);
    double distance = norm(delta);
    if (distance <= 0)
      continue;
    const double limit = std::min(distance, range);
    const auto n = std::max<std::size_t>(1, std::size_t(std::ceil(limit / (resolution_ * .5))));
    for (std::size_t i = 0; i < n; ++i) {
      Index idx{};
      if (index(add(origin, mul(delta, (double(i) / n) * limit / distance)), idx))
        evidence[flat(idx)] |= 1;
    }
    Index idx{};
    if (distance <= range && index(p, idx))
      evidence[flat(idx)] |= 2;
  }
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    if (evidence[i] & 2)
      cells_[i] = Occupancy::kOccupied;
    else if (evidence[i] & 1)
      cells_[i] = Occupancy::kFree;
  }
}
VoxelMap::VoxelMap(Vec3 lo, Vec3 hi, double r, Occupancy initial)
    : minimum_(lo), maximum_(hi), resolution_(r) {
  if (!finite(lo) || !finite(hi) || !std::isfinite(r) || r <= 0)
    throw std::invalid_argument("invalid voxel map");
  for (std::size_t i = 0; i < 3; ++i) {
    if (hi[i] <= lo[i])
      throw std::invalid_argument("invalid voxel bounds");
    shape_[i] = static_cast<std::size_t>(std::ceil((hi[i] - lo[i]) / r));
  }
  cells_.assign(shape_[0] * shape_[1] * shape_[2], initial);
}
std::size_t VoxelMap::flat(const std::array<std::size_t, 3>& i) const noexcept {
  return (i[0] * shape_[1] + i[1]) * shape_[2] + i[2];
}
bool VoxelMap::index(const Vec3& p, std::array<std::size_t, 3>& out) const noexcept {
  if (!finite(p))
    return false;
  for (std::size_t i = 0; i < 3; ++i) {
    if (p[i] < minimum_[i] || p[i] >= maximum_[i])
      return false;
    out[i] = static_cast<std::size_t>(std::floor((p[i] - minimum_[i]) / resolution_));
    if (out[i] >= shape_[i])
      return false;
  }
  return true;
}
bool VoxelMap::contains(const Vec3& p) const noexcept {
  std::array<std::size_t, 3> i{};
  return index(p, i);
}
bool VoxelMap::world_to_index(const Vec3& p, Index& i) const noexcept {
  return index(p, i);
}
Vec3 VoxelMap::index_center(const Index& i) const {
  if (i[0] >= shape_[0] || i[1] >= shape_[1] || i[2] >= shape_[2])
    throw std::out_of_range("voxel index outside map");
  return {minimum_[0] + (i[0] + .5) * resolution_, minimum_[1] + (i[1] + .5) * resolution_,
          minimum_[2] + (i[2] + .5) * resolution_};
}
Occupancy VoxelMap::at(const Index& i) const {
  if (i[0] >= shape_[0] || i[1] >= shape_[1] || i[2] >= shape_[2])
    throw std::out_of_range("voxel index outside map");
  return cells_[flat(i)];
}
void VoxelMap::set_index(const Index& i, Occupancy v) {
  if (i[0] >= shape_[0] || i[1] >= shape_[1] || i[2] >= shape_[2])
    throw std::out_of_range("voxel index outside map");
  cells_[flat(i)] = v;
}
bool VoxelMap::occupied(const Vec3& p, bool unknown) const noexcept {
  std::array<std::size_t, 3> i{};
  if (!index(p, i))
    return true;
  const auto v = cells_[flat(i)];
  return v == Occupancy::kOccupied || (unknown && v == Occupancy::kUnknown);
}
void VoxelMap::set(const Vec3& p, Occupancy v) {
  std::array<std::size_t, 3> i{};
  if (!index(p, i))
    throw std::out_of_range("point outside map");
  cells_[flat(i)] = v;
}
void VoxelMap::fill(Occupancy v) noexcept {
  std::fill(cells_.begin(), cells_.end(), v);
}
void VoxelMap::integrate_ray(const Vec3& origin, const Vec3& endpoint, double max_range, bool hit) {
  if (!finite(origin) || !finite(endpoint) || max_range <= 0)
    throw std::invalid_argument("invalid occupancy ray");
  const Vec3 delta = sub(endpoint, origin);
  const double distance = norm(delta);
  if (distance <= 0)
    return;
  const double limit = std::min(distance, max_range);
  const auto samples =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(limit / (resolution_ * .5))));
  for (std::size_t i = 0; i < samples; ++i) {
    Index idx{};
    if (index(add(origin, mul(delta, (double(i) / samples) * limit / distance)), idx))
      set_index(idx, Occupancy::kFree);
  }
  if (hit && distance <= max_range) {
    Index idx{};
    if (index(endpoint, idx))
      set_index(idx, Occupancy::kOccupied);
  }
}
VoxelMap VoxelMap::inflated(double radius) const {
  if (radius < 0)
    throw std::invalid_argument("negative inflation radius");
  VoxelMap out = *this;
  const int cells = static_cast<int>(std::ceil(radius / resolution_));
  for (std::size_t x = 0; x < shape_[0]; ++x)
    for (std::size_t y = 0; y < shape_[1]; ++y)
      for (std::size_t z = 0; z < shape_[2]; ++z)
        if (at({x, y, z}) == Occupancy::kOccupied)
          for (int dx = -cells; dx <= cells; ++dx)
            for (int dy = -cells; dy <= cells; ++dy)
              for (int dz = -cells; dz <= cells; ++dz) {
                if (std::sqrt(double(dx * dx + dy * dy + dz * dz)) * resolution_ > radius)
                  continue;
                long nx = long(x) + dx, ny = long(y) + dy, nz = long(z) + dz;
                if (nx >= 0 && ny >= 0 && nz >= 0 && nx < long(shape_[0]) && ny < long(shape_[1]) &&
                    nz < long(shape_[2]))
                  out.set_index({std::size_t(nx), std::size_t(ny), std::size_t(nz)},
                                Occupancy::kOccupied);
              }
  return out;
}
void VoxelMap::fill_box(const Vec3& lo, const Vec3& hi, Occupancy v) {
  for (std::size_t x = 0; x < shape_[0]; ++x)
    for (std::size_t y = 0; y < shape_[1]; ++y)
      for (std::size_t z = 0; z < shape_[2]; ++z) {
        Vec3 p{minimum_[0] + (x + .5) * resolution_, minimum_[1] + (y + .5) * resolution_,
               minimum_[2] + (z + .5) * resolution_};
        if (p[0] >= lo[0] && p[0] <= hi[0] && p[1] >= lo[1] && p[1] <= hi[1] && p[2] >= lo[2] &&
            p[2] <= hi[2])
          cells_[flat({x, y, z})] = v;
      }
}
bool VoxelMap::segment_occupied(const Vec3& a, const Vec3& b, double spacing, bool unknown,
                                std::size_t* checks) const {
  if (spacing <= 0)
    throw std::invalid_argument("spacing must be positive");
  const auto n =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(norm(sub(b, a)) / spacing)));
  for (std::size_t i = 0; i <= n; ++i) {
    if (checks)
      ++*checks;
    if (occupied(add(a, mul(sub(b, a), double(i) / n)), unknown))
      return true;
  }
  return false;
}
std::vector<Vec3> VoxelMap::occupied_centers() const {
  std::vector<Vec3> out;
  for (std::size_t x = 0; x < shape_[0]; ++x)
    for (std::size_t y = 0; y < shape_[1]; ++y)
      for (std::size_t z = 0; z < shape_[2]; ++z)
        if (cells_[flat({x, y, z})] == Occupancy::kOccupied)
          out.push_back({minimum_[0] + (x + .5) * resolution_, minimum_[1] + (y + .5) * resolution_,
                         minimum_[2] + (z + .5) * resolution_});
  return out;
}
}  // namespace robotics::navigation

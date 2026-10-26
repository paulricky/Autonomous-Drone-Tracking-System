#include "ece3162_core/voxel_map.hpp"
#include <algorithm>
#include <stdexcept>

namespace ece3162::navigation {
VoxelMap::VoxelMap(Vec3 lo, Vec3 hi, double r, Occupancy initial):minimum_(lo),maximum_(hi),resolution_(r){
  if(!finite(lo)||!finite(hi)||!std::isfinite(r)||r<=0) throw std::invalid_argument("invalid voxel map");
  for(std::size_t i=0;i<3;++i){if(hi[i]<=lo[i])throw std::invalid_argument("invalid voxel bounds");shape_[i]=static_cast<std::size_t>(std::ceil((hi[i]-lo[i])/r));}
  cells_.assign(shape_[0]*shape_[1]*shape_[2],initial);
}
std::size_t VoxelMap::flat(const std::array<std::size_t,3>& i) const noexcept{return (i[0]*shape_[1]+i[1])*shape_[2]+i[2];}
bool VoxelMap::index(const Vec3& p,std::array<std::size_t,3>& out)const noexcept{
  if(!finite(p))return false;
  for(std::size_t i=0;i<3;++i){if(p[i]<minimum_[i]||p[i]>=maximum_[i])return false;out[i]=static_cast<std::size_t>(std::floor((p[i]-minimum_[i])/resolution_));if(out[i]>=shape_[i])return false;}return true;
}
bool VoxelMap::contains(const Vec3& p)const noexcept{std::array<std::size_t,3> i{};return index(p,i);}
bool VoxelMap::occupied(const Vec3& p,bool unknown)const noexcept{std::array<std::size_t,3> i{};if(!index(p,i))return true;const auto v=cells_[flat(i)];return v==Occupancy::kOccupied||(unknown&&v==Occupancy::kUnknown);}
void VoxelMap::set(const Vec3& p,Occupancy v){std::array<std::size_t,3> i{};if(!index(p,i))throw std::out_of_range("point outside map");cells_[flat(i)]=v;}
void VoxelMap::fill(Occupancy v)noexcept{std::fill(cells_.begin(),cells_.end(),v);}
void VoxelMap::fill_box(const Vec3& lo,const Vec3& hi,Occupancy v){for(std::size_t x=0;x<shape_[0];++x)for(std::size_t y=0;y<shape_[1];++y)for(std::size_t z=0;z<shape_[2];++z){Vec3 p{minimum_[0]+(x+.5)*resolution_,minimum_[1]+(y+.5)*resolution_,minimum_[2]+(z+.5)*resolution_};if(p[0]>=lo[0]&&p[0]<=hi[0]&&p[1]>=lo[1]&&p[1]<=hi[1]&&p[2]>=lo[2]&&p[2]<=hi[2])cells_[flat({x,y,z})]=v;}}
bool VoxelMap::segment_occupied(const Vec3&a,const Vec3&b,double spacing,bool unknown,std::size_t* checks)const{if(spacing<=0)throw std::invalid_argument("spacing must be positive");const auto n=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(norm(sub(b,a))/spacing)));for(std::size_t i=0;i<=n;++i){if(checks)++*checks;if(occupied(add(a,mul(sub(b,a),double(i)/n)),unknown))return true;}return false;}
std::vector<Vec3> VoxelMap::occupied_centers()const{std::vector<Vec3> out;for(std::size_t x=0;x<shape_[0];++x)for(std::size_t y=0;y<shape_[1];++y)for(std::size_t z=0;z<shape_[2];++z)if(cells_[flat({x,y,z})]==Occupancy::kOccupied)out.push_back({minimum_[0]+(x+.5)*resolution_,minimum_[1]+(y+.5)*resolution_,minimum_[2]+(z+.5)*resolution_});return out;}
}  // namespace ece3162::navigation

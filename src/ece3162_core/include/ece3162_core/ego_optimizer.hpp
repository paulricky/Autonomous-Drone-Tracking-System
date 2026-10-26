#pragma once
#include "ece3162_core/bspline.hpp"
#include <memory>

namespace ece3162::navigation {
struct EgoConfig {int degree{3};double clearance{0.5},smoothness_weight{1.0},collision_weight{100.0},guidance_weight{2.0},feasibility_weight{10.0};double gradient_tolerance{1e-5},initial_step{0.05},sample_dt{0.03};std::size_t max_iterations{120};};
struct ObjectiveBreakdown {double smoothness{},collision{},guidance{},feasibility{};double total()const{return smoothness+collision+guidance+feasibility;}};
struct EgoResult {bool success{};FailureCode code{FailureCode::kOptimizerFailure};std::string message;std::shared_ptr<BSpline> trajectory;ObjectiveBreakdown initial,final;FeasibilityReport feasibility;std::size_t iterations{},objective_evaluations{};double optimization_time_s{};};
class EgoOptimizer {
 public:
  explicit EgoOptimizer(EgoConfig config={});
  ObjectiveBreakdown objective(const std::vector<Vec3>& points,const std::vector<Vec3>& guidance,const VoxelMap& map,double duration,const DynamicLimits& limits,std::vector<Vec3>* gradient=nullptr)const;
  EgoResult optimize(const std::vector<Vec3>& guidance,const VoxelMap&,double duration,const DynamicLimits&)const;
 private:EgoConfig config_;
};
}  // namespace ece3162::navigation

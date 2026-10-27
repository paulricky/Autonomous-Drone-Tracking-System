#include <gtest/gtest.h>

#include "ece3162_core/bspline.hpp"
#include "ece3162_core/dynamics.hpp"
#include "ece3162_core/ego_optimizer.hpp"
#include "ece3162_core/kinodynamic_astar.hpp"

using namespace ece3162::navigation;
TEST(VoxelMap, BatchEndpointsWinAndClearingRequiresEvidence) {
  VoxelMap map({0, 0, 0}, {4, 4, 4}, .25);
  const Vec3 origin{.1, .1, .1};
  map.integrate_cloud(origin, {{1.1, .1, .1}, {2.1, .1, .1}}, 3);
  EXPECT_TRUE(map.occupied({1.1, .1, .1}));
  EXPECT_TRUE(map.occupied({2.1, .1, .1}));
  map.integrate_cloud(origin, {{2.1, .1, .1}}, 3);
  EXPECT_FALSE(map.occupied({1.1, .1, .1}));
  EXPECT_TRUE(map.occupied({2.1, .1, .1}));
  EXPECT_EQ(map.at({15, 15, 15}), Occupancy::kUnknown);
}
TEST(KinodynamicAStar, PropagationAndPlan) {
  auto next = KinodynamicAStar::propagate({{0, 0, 1}, {.2, 0, 0}, 0}, {{1, 0, 0}, .5});
  EXPECT_NEAR(next.position[0], .225, 1e-12);
  EXPECT_NEAR(next.velocity[0], .7, 1e-12);
  VoxelMap map({-1, -2, 0}, {6, 2, 3}, .25, Occupancy::kFree);
  KinodynamicConfig c;
  c.max_velocity = 1;
  c.max_acceleration = 1;
  c.timeout_s = 3;
  c.max_expansions = 20000;
  auto result = KinodynamicAStar(c).plan({{0, 0, 1}, {0, 0, 0}, 0}, {3, 0, 1}, map);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LE(norm(sub(result.states.back().position, {3, 0, 1})), c.goal_position_tolerance);
  for (const auto& s : result.states)
    EXPECT_LE(norm(s.velocity), 1 + 1e-9);
}
TEST(KinodynamicAStar, ObstacleAndFailure) {
  VoxelMap map({-1, -2, 0}, {5, 2, 3}, .25, Occupancy::kFree);
  map.fill_box({1, -.5, 0}, {2, .5, 3});
  KinodynamicConfig c;
  c.heuristic_weight = 2;
  c.timeout_s = 5;
  c.max_expansions = 40000;
  auto r = KinodynamicAStar(c).plan({{0, 0, 1}, {0, 0, 0}, 0}, {3, 0, 1}, map);
  ASSERT_TRUE(r.success) << r.message;
  for (std::size_t i = 1; i < r.states.size(); ++i)
    EXPECT_FALSE(map.segment_occupied(r.states[i - 1].position, r.states[i].position, .04));
  VoxelMap blocked({-1, -2, 0}, {5, 2, 3}, .25, Occupancy::kFree);
  blocked.fill_box({1, -2, 0}, {2, 2, 3});
  c.timeout_s = .5;
  EXPECT_FALSE(KinodynamicAStar(c).plan({{0, 0, 1}, {0, 0, 0}, 0}, {3, 0, 1}, blocked).success);
}
TEST(BSpline, AnalyticDerivativesAndValidation) {
  std::vector<Vec3> cp{{0, 0, 1}, {.8, 0, 1}, {1.6, 0, 1}, {2.4, 0, 1}, {3, 0, 1}};
  BSpline s(cp, 5, 3);
  EXPECT_EQ(s.position(0), cp.front());
  EXPECT_EQ(s.position(5), cp.back());
  double e = 1e-5;
  auto numeric = mul(sub(s.position(2 + e), s.position(2 - e)), 1 / (2 * e));
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(s.velocity(2)[i], numeric[i], 1e-6);
  VoxelMap map({-1, -2, 0}, {6, 2, 3}, .25, Occupancy::kFree);
  EXPECT_TRUE(s.validate(map, {2, 2}, .02).valid);
}
TEST(EgoOptimizer, GradientAndOptimization) {
  VoxelMap map({-1, -2, 0}, {6, 2, 3}, .25, Occupancy::kFree);
  std::vector<Vec3> g{{0, 0, 1}, {.7, .2, 1}, {1.4, .4, 1}, {2.1, .2, 1}, {3, 0, 1}};
  EgoOptimizer opt;
  std::vector<Vec3> analytic;
  auto c = opt.objective(g, g, map, 5, {2, 2}, &analytic);
  double e = 1e-6;
  for (std::size_t i = 1; i + 1 < g.size(); ++i)
    for (int axis = 0; axis < 3; ++axis) {
      auto plus = g, minus = g;
      plus[i][axis] += e;
      minus[i][axis] -= e;
      double numeric = (opt.objective(plus, g, map, 5, {2, 2}).total() -
                        opt.objective(minus, g, map, 5, {2, 2}).total()) /
                       (2 * e);
      EXPECT_NEAR(analytic[i][axis], numeric, 2e-4);
    }
  auto r = opt.optimize(g, map, 5, {2, 2});
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_LE(r.final.total(), c.total() + 1e-9);
}
TEST(Dynamics, ClosedLoopStateEvolution) {
  DoubleIntegrator sim({{0, 0, 1}, {0, 0, 0}, 0}, {.02, 2, 2, 0, 0, 0, 0});
  for (int i = 0; i < 50; ++i)
    sim.step({1, 0, 0});
  EXPECT_NEAR(sim.ground_truth().position[0], .5, 1e-9);
  EXPECT_NEAR(sim.ground_truth().velocity[0], 1, 1e-9);
}
TEST(VoxelMap, RayIntegrationAndInflation) {
  VoxelMap map({0, 0, 0}, {4, 4, 4}, .25, Occupancy::kUnknown);
  map.integrate_ray({.1, .1, .1}, {2.1, .1, .1}, 3, true);
  EXPECT_FALSE(map.occupied({1, .1, .1}));
  EXPECT_TRUE(map.occupied({2.1, .1, .1}));
  auto inflated = map.inflated(.3);
  EXPECT_TRUE(inflated.occupied({2.1, .35, .1}));
  EXPECT_EQ(map.at({15, 15, 15}), Occupancy::kUnknown);
}

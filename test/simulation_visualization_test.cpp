#include "features/simulation/lidar_sim.hpp"
#include "features/simulation/unicycle_model.hpp"
#include "features/visualization/visualizer.hpp"

#include <cstdint>
#include <gtest/gtest.h>
#include <vector>

namespace ad::simulation {
namespace {

[[nodiscard]] auto makeObstacleMap() -> types::MapData {
  const int width = 4;
  const int height = 3;
  const double resolution = 1.0;
  auto grid = std::vector<std::int8_t>(static_cast<std::size_t>(width * height), 0);
  grid[static_cast<std::size_t>(1 * width + 2)] = 1; // obstacle at (2,1)
  return types::MapData{width, height, resolution, grid};
}

} // namespace

TEST(UnicycleModelTest, PropagateClampsAndIntegrates) {
  const UnicycleModel model;
  const MotionState state{{0.0, 0.0, 0.0}, {0.0, 0.0}};
  const types::Twist command{10.0, 10.0};

  const auto result = model.propagate(state, command, 0.5);
  ASSERT_TRUE(result.has_value());

  EXPECT_NEAR(result->pose.x, 2.5, 1e-6);
  EXPECT_NEAR(result->pose.y, 0.0, 1e-6);
  EXPECT_NEAR(result->pose.theta, 1.5, 1e-6);
  EXPECT_DOUBLE_EQ(result->twist.v, 5.0);
  EXPECT_DOUBLE_EQ(result->twist.w, 3.0);
}

TEST(UnicycleModelTest, RejectsNonPositiveDelta) {
  const UnicycleModel model;
  const MotionState state{{0.0, 0.0, 0.0}, {0.0, 0.0}};
  const types::Twist command{1.0, 0.5};

  const auto result = model.propagate(state, command, 0.0);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ErrorCode::InvalidInput);
}

TEST(LidarSimTest, DetectsObstacleAlongHeadingRay) {
  const LidarSim lidar;
  const auto map = makeObstacleMap();
  const types::Pose pose{0.5, 1.0, 0.0};

  const auto scan = lidar.simulate(map, pose);
  ASSERT_TRUE(scan.has_value());
  ASSERT_EQ(scan->size(), 4U);
  EXPECT_NEAR((*scan)[2], 2.0, 1e-6);
}

TEST(LidarSimTest, RejectsMismatchedGrid) {
  const LidarSim lidar;
  const types::MapData map{2, 2, 1.0, {0, 0, 0}};
  const types::Pose pose{0.0, 0.0, 0.0};

  const auto scan = lidar.simulate(map, pose);
  ASSERT_FALSE(scan.has_value());
  EXPECT_EQ(scan.error().code, ErrorCode::SizeMismatch);
}

TEST(VisualizerTest, RejectsEmptyPath) {
  const visualization::Visualizer viz;
  const auto status = viz.renderPath({});
  ASSERT_FALSE(status.has_value());
  EXPECT_EQ(status.error().code, ErrorCode::EmptyCollection);
}

TEST(VisualizerTest, RendersScanSummary) {
  const visualization::Visualizer viz;
  const std::vector<double> ranges{1.0, 2.0, 1.5};
  const types::MapData map{2, 2, 1.0, {0, 0, 0, 0}};
  const types::Pose pose{0.0, 0.0, 0.0};

  const auto frameStatus = viz.renderFrame(map);
  ASSERT_TRUE(frameStatus.has_value());

  const auto status = viz.renderScan(pose, ranges);
  EXPECT_TRUE(status.has_value());
}

} // namespace ad::simulation

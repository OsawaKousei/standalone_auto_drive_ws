#include "features/planning/dijkstra_planner.hpp"
#include "shared/types.hpp"

#include <gtest/gtest.h>

namespace ad::planning {
namespace {

[[nodiscard]] auto makeMap() -> types::MapData {
  const int width = 5;
  const int height = 4;
  const double resolution = 1.0;
  auto grid = std::vector<std::int8_t>(static_cast<std::size_t>(width * height), 0);
  grid[static_cast<std::size_t>(1 * width + 2)] = 1;
  grid[static_cast<std::size_t>(2 * width + 2)] = 1;
  return types::MapData{width, height, resolution, grid};
}

} // namespace

TEST(DijkstraPlannerTest, ReturnsPathOnFreeSpace) {
  const auto map = makeMap();
  const DijkstraPlanner planner;
  const types::Pose start{0.5, 0.5, 0.0};
  const types::Pose goal{4.5, 3.5, 0.0};

  const auto result = planner.plan(map, start, goal);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->empty());
  EXPECT_DOUBLE_EQ(result->front().x, 0.5);
  EXPECT_DOUBLE_EQ(result->front().y, 0.5);
  EXPECT_DOUBLE_EQ(result->back().x, 4.5);
  EXPECT_DOUBLE_EQ(result->back().y, 3.5);
}

TEST(DijkstraPlannerTest, ReturnsSinglePointWhenStartEqualsGoal) {
  const auto map = makeMap();
  const DijkstraPlanner planner;
  const types::Pose start{1.5, 0.5, 0.0};

  const auto result = planner.plan(map, start, start);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1U);
  EXPECT_DOUBLE_EQ((*result)[0].x, 1.5);
  EXPECT_DOUBLE_EQ((*result)[0].y, 0.5);
}

TEST(DijkstraPlannerTest, RejectsStartInsideObstacle) {
  const auto map = makeMap();
  const DijkstraPlanner planner;
  const types::Pose start{2.5, 1.5, 0.0};
  const types::Pose goal{0.5, 0.5, 0.0};

  const auto result = planner.plan(map, start, goal);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ErrorCode::InvalidInput);
}

TEST(DijkstraPlannerTest, RejectsMismatchedGrid) {
  const types::MapData map{2, 2, 1.0, {0, 0, 0}};
  const DijkstraPlanner planner;
  const types::Pose start{0.5, 0.5, 0.0};
  const types::Pose goal{1.5, 1.5, 0.0};

  const auto result = planner.plan(map, start, goal);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ErrorCode::SizeMismatch);
}

} // namespace ad::planning

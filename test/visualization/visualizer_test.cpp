#include "features/visualization/visualizer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace {
using ad::visualization::Visualizer;

// Deliberately asymmetric map to exercise rectangular geometry and lower origin.
auto prepareMap() -> ad::Result<Visualizer::PreparedMap> {
  return Visualizer::prepareMap({4, 3, 0.5, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}});
}

TEST(Visualizer, ValidatesMapBeforeCreatingGpu) {
  const auto invalid = Visualizer::prepareMap({0, 1, 0.5, {}});
  ASSERT_FALSE(invalid);
  EXPECT_EQ(invalid.error().code, ad::ErrorCode::InvalidInput);
  const auto mismatch = Visualizer::prepareMap({2, 2, 0.5, {0}});
  ASSERT_FALSE(mismatch);
  EXPECT_EQ(mismatch.error().code, ad::ErrorCode::SizeMismatch);
  const auto nonfinite =
      Visualizer::prepareMap({1, 1, std::numeric_limits<double>::infinity(), {0}});
  ASSERT_FALSE(nonfinite);
  EXPECT_EQ(nonfinite.error().code, ad::ErrorCode::InvalidInput);
}

TEST(Visualizer, PreparedMapCopyOutlivesOriginalAndWorksAcrossVisualizers) {
  auto copy = std::optional<Visualizer::PreparedMap>{};
  {
    const auto original = prepareMap();
    ASSERT_TRUE(original);
    copy.emplace(*original);
  }
  const Visualizer first;
  const Visualizer second;
  ASSERT_TRUE(first.renderFrame(*copy));
  ASSERT_TRUE(second.renderFrame(*copy));
  EXPECT_EQ(copy->geometry.width, 4);
  EXPECT_EQ(copy->geometry.height, 3);
  EXPECT_DOUBLE_EQ(copy->geometry.resolution, 0.5);
}

TEST(Visualizer, ConvertsErrorsAndPreservesEmptyCollectionCodes) {
  const Visualizer viz;
  const Visualizer::MapGeometry geometry{4, 3, 0.5};
  const auto noFrame = viz.renderMarker({0.5, 0.5}, geometry, 10, "red");
  ASSERT_FALSE(noFrame);
  EXPECT_EQ(noFrame.error().code, ad::ErrorCode::InvalidInput);
  EXPECT_FALSE(noFrame.error().message.empty());
  const auto path = viz.renderPath({}, geometry);
  const auto points = viz.renderPoints({}, geometry, 10, "green");
  const auto robot = viz.renderRobot({0, 0, 0}, {}, geometry);
  const auto scan = viz.renderScan({0, 0, 0}, {{}, 0, 0.1, 2}, geometry);
  for (const auto *status : {&path, &points, &robot, &scan}) {
    ASSERT_FALSE(*status);
    EXPECT_EQ(status->error().code, ad::ErrorCode::EmptyCollection);
  }
  EXPECT_FALSE(viz.saveFigure(""));
}

TEST(Visualizer, SupportsAllUsedStylesAndRejectsInvalidData) {
  const auto prepared = prepareMap();
  ASSERT_TRUE(prepared);
  const Visualizer viz;
  ASSERT_TRUE(viz.renderFrame(*prepared));
  const auto &geometry = prepared->geometry;
  const std::vector<ad::types::Point> path{{0.25, 0.25}, {0.75, 0.75}, {1.75, 1.25}};
  for (const auto style : {"b-", "r-", "m-", "k--", "c--"}) {
    ASSERT_TRUE(viz.renderPath(path, geometry, style));
  }
  for (const auto color : {"red", "green", "blue", "black", "cyan", "magenta", "lime"}) {
    ASSERT_TRUE(viz.renderMarker(path.front(), geometry, 26, color));
  }
  const auto invalidStyle = viz.renderPath(path, geometry, "unknown");
  ASSERT_FALSE(invalidStyle);
  EXPECT_EQ(invalidStyle.error().code, ad::ErrorCode::InvalidInput);
  const auto wrongMap = viz.renderPath(path, {5, 3, 0.5});
  ASSERT_FALSE(wrongMap);
  EXPECT_EQ(wrongMap.error().code, ad::ErrorCode::InvalidInput);
  const auto invalidPoint =
      viz.renderMarker({std::numeric_limits<double>::quiet_NaN(), 0}, geometry, 10, "red");
  ASSERT_FALSE(invalidPoint);
  EXPECT_EQ(invalidPoint.error().code, ad::ErrorCode::InvalidInput);
  ASSERT_TRUE(
      viz.renderRobot({0.75, 0.75, 0.4}, {{{-0.1, -0.1}, {0.2, 0}, {-0.1, 0.1}}}, geometry));
  ASSERT_TRUE(viz.renderPoints(path, geometry, 10, "green"));
  ASSERT_TRUE(viz.renderScan({0.75, 0.75, 0},
                             {{0.2, -1, std::numeric_limits<double>::infinity(), 3}, -0.5, 0.5, 2},
                             geometry));
}

TEST(Visualizer, MoveTransfersSceneAndAllowsReinitializingSource) {
  const auto prepared = prepareMap();
  ASSERT_TRUE(prepared);
  Visualizer source;
  ASSERT_TRUE(source.renderFrame(*prepared));
  Visualizer moved{std::move(source)};
  ASSERT_TRUE(moved.renderMarker({0.5, 0.5}, prepared->geometry, 10, "red"));
  ASSERT_TRUE(source.renderFrame(*prepared));
  moved = std::move(source);
  ASSERT_TRUE(moved.renderMarker({0.5, 0.5}, prepared->geometry, 10, "red"));
}

class VisualizerOutput : public ::testing::Test {
protected:
  void SetUp() override {
    directory = std::filesystem::temp_directory_path() /
                ("ad-visualizer-test-" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
  }
  void TearDown() override { std::filesystem::remove_all(directory); }
  std::filesystem::path directory;
};

TEST_F(VisualizerOutput, Png) {
  const auto prepared = prepareMap();
  ASSERT_TRUE(prepared);
  const Visualizer viz;
  ASSERT_TRUE(viz.renderFrame(*prepared));
  const auto output = directory / "scene.png";
  const auto save = viz.saveFigure(output.string());
  ASSERT_TRUE(save) << save.error().message;
  std::ifstream input{output, std::ios::binary};
  std::array<unsigned char, 8> signature{};
  input.read(reinterpret_cast<char *>(signature.data()), signature.size());
  EXPECT_EQ(signature, (std::array<unsigned char, 8>{137, 80, 78, 71, 13, 10, 26, 10}));
  ASSERT_TRUE(viz.renderMarker({0.5, 0.5}, prepared->geometry, 26, "red"));
  ASSERT_TRUE(viz.saveFigure(output.string()));
  const auto failure = viz.saveFigure((directory / "missing" / "scene.png").string());
  ASSERT_FALSE(failure);
  EXPECT_EQ(failure.error().code, ad::ErrorCode::BackendFailure);
  EXPECT_FALSE(failure.error().message.empty());
}

TEST_F(VisualizerOutput, Window) {
  const auto prepared = prepareMap();
  ASSERT_TRUE(prepared);
  const Visualizer viz;
  for (int frame = 0; frame < 5; ++frame) {
    ASSERT_TRUE(viz.renderFrame(*prepared));
    ASSERT_TRUE(viz.renderRobot({0.75 + frame * 0.1, 0.75, 0.4},
                                {{{-0.1, -0.1}, {0.2, 0}, {-0.1, 0.1}}}, prepared->geometry));
    const auto present = viz.presentFrame();
    ASSERT_TRUE(present) << present.error().message;
  }
  ASSERT_TRUE(viz.saveFigure((directory / "window.png").string()));
  ASSERT_TRUE(viz.presentFrame());
}
} // namespace

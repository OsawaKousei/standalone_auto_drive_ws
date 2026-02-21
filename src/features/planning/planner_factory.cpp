#include "planner_factory.hpp"

#include "astar_planner.hpp"
#include "dijkstra_planner.hpp"
#include "grid_collision_checker.hpp"

namespace ad::planning {

namespace {

[[nodiscard]] auto
parseCollisionCheckerType(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::string> {
  if (!configDoc.has_value()) {
    return std::string{"grid"};
  }

  const auto raw = configDoc->findRaw("", "collision_checker");
  if (!raw) {
    return std::string{"grid"};
  }

  const auto parsed = ::ad::config::parseQuotedString(*raw);
  if (!parsed) {
    return tl::make_unexpected(parsed.error());
  }
  return *parsed;
}

} // namespace

auto createPlannerFromConfig(std::string_view algorithm, const types::MapData &map,
                             const types::Footprint &footprint,
                             const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<PlannerComponents> {
  const auto checkerType = parseCollisionCheckerType(configDoc);
  if (!checkerType) {
    return tl::make_unexpected(checkerType.error());
  }

  std::unique_ptr<ICollisionChecker> collisionChecker;
  if (*checkerType == "grid") {
    auto checker = GridCollisionChecker::create(map, footprint);
    if (!checker) {
      return tl::make_unexpected(checker.error());
    }
    collisionChecker = std::move(*checker);
  } else {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported planner collision checker: " + *checkerType});
  }

  std::unique_ptr<IPlanner> planner;
  if (algorithm == "astar") {
    planner = std::unique_ptr<IPlanner>{new AStarPlanner{*collisionChecker}};
  } else if (algorithm == "dijkstra") {
    planner = std::unique_ptr<IPlanner>{new DijkstraPlanner{*collisionChecker}};
  } else {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported planning algorithm: " + std::string{algorithm}});
  }

  return PlannerComponents{.collisionChecker = std::move(collisionChecker),
                           .planner = std::move(planner)};
}

} // namespace ad::planning

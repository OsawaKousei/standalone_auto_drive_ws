#include "planner_factory.hpp"

#include "astar_planner.hpp"
#include "dijkstra_planner.hpp"

namespace ad::planning {

auto createPlannerFromConfig(std::string_view algorithm, const ICollisionChecker &collisionChecker)
    -> Result<std::unique_ptr<IPlanner>> {
  if (algorithm == "astar") {
    return std::unique_ptr<IPlanner>{new AStarPlanner{collisionChecker}};
  }
  if (algorithm == "dijkstra") {
    return std::unique_ptr<IPlanner>{new DijkstraPlanner{collisionChecker}};
  }
  return tl::make_unexpected(
      Error{.code = ErrorCode::InvalidInput,
            .message = "Unsupported planning algorithm: " + std::string{algorithm}});
}

} // namespace ad::planning

#pragma once

#include "i_planner.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace ad::planning {

class DijkstraPlanner final : public IPlanner {
public:
  DijkstraPlanner() = default;

  [[nodiscard]] auto plan(const types::MapData &map, const types::Pose &start,
                          const types::Pose &goal) const -> Result<types::Path> override;

private:
  struct StartGoalInfo {
    const std::size_t startIndex;
    const std::size_t goalIndex;
    const types::Point startCenter;
  };

  [[nodiscard]] auto validateInputs(const types::MapData &map, const types::Pose &start,
                                    const types::Pose &goal) const -> Result<StartGoalInfo>;
  [[nodiscard]] auto computePrevious(const types::MapData &map,
                                     const StartGoalInfo &startGoal) const
      -> Result<std::vector<std::optional<std::size_t>>>;
  [[nodiscard]] auto buildPath(const types::MapData &map,
                               const std::vector<std::optional<std::size_t>> &previous,
                               const StartGoalInfo &startGoal) const -> types::Path;
};

} // namespace ad::planning

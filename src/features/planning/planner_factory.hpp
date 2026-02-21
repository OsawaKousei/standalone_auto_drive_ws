#pragma once

#include "i_collision_checker.hpp"
#include "i_planner.hpp"

#include "../../shared/result.hpp"
#include "../../shared/text_config.hpp"
#include "../../shared/types.hpp"

#include <memory>
#include <optional>
#include <string_view>

namespace ad::planning {

struct PlannerComponents {
  std::unique_ptr<ICollisionChecker> collisionChecker;
  std::unique_ptr<IPlanner> planner;
};

[[nodiscard]] auto createPlannerFromConfig(std::string_view algorithm, const types::MapData &map,
                                           const types::Footprint &footprint,
                                           const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<PlannerComponents>;

} // namespace ad::planning

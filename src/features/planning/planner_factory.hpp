#pragma once

#include "i_collision_checker.hpp"
#include "i_planner.hpp"

#include "../../shared/result.hpp"

#include <memory>
#include <string_view>

namespace ad::planning {

[[nodiscard]] auto createPlannerFromConfig(std::string_view algorithm,
                                           const ICollisionChecker &collisionChecker)
    -> Result<std::unique_ptr<IPlanner>>;

} // namespace ad::planning

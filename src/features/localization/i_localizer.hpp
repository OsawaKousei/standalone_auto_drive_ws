#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <array>

namespace ad::localization {

struct LocalizerEstimate {
  const types::Pose pose;
  const std::array<double, 9> covariance;
  const double score;
};

class ILocalizer {
public:
  virtual ~ILocalizer() = default;
  [[nodiscard]] virtual Status reset(const types::Pose &initialPose,
                                     const std::array<double, 9> &initialCovariance) = 0;
  [[nodiscard]] virtual Status predict(const types::Twist &control, double dt) = 0;
  [[nodiscard]] virtual Status update(const types::LidarScan &scan, const types::MapData &map) = 0;
  [[nodiscard]] virtual Result<LocalizerEstimate> estimate() const = 0;
};

} // namespace ad::localization

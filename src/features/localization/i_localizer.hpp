#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include "localization_config.hpp"

namespace ad::localization {

struct LocalizerEstimate {
  const types::Pose pose;
  const CovarianceMatrix covariance;
  const double score;
};

class ILocalizer {
public:
  virtual ~ILocalizer() = default;
  ILocalizer() = default;
  ILocalizer(const ILocalizer &) = delete;
  auto operator=(const ILocalizer &) -> ILocalizer & = delete;
  ILocalizer(ILocalizer &&) = delete;
  auto operator=(ILocalizer &&) -> ILocalizer & = delete;
  [[nodiscard]] virtual auto reset(const types::Pose &initialPose,
                                   const CovarianceMatrix &initialCovariance) -> Status = 0;
  [[nodiscard]] virtual auto predict(const types::Twist &control, double deltaT) -> Status = 0;
  [[nodiscard]] virtual auto update(const types::LidarScan &scan, const types::MapData &map)
      -> Status = 0;
  [[nodiscard]] virtual auto estimate() const -> Result<LocalizerEstimate> = 0;
};

} // namespace ad::localization

#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <random>

namespace ad::simulation {

struct OdometrySensorConfig {
  const double forwardNoiseStddev;
  const double lateralNoiseStddev;
  const double thetaNoiseStddev;
};

class OdometrySensor {
public:
  explicit OdometrySensor(OdometrySensorConfig config);

  [[nodiscard]] auto measure(const types::Pose &previousPose, const types::Pose &currentPose) const
      -> Result<types::OdometryDelta>;

private:
  const OdometrySensorConfig config_;
  mutable std::mt19937 generator_;
};

} // namespace ad::simulation

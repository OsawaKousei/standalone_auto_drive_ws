#pragma once

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <random>

namespace ad::simulation {

struct OdometrySensorConfig {
  const double forwardNoiseStddev;
  const double lateralNoiseStddev;
  const double thetaNoiseStddev;
  const int seed;
};

namespace config {

constexpr double kDefaultForwardNoiseStddev = 0.0;
constexpr double kDefaultLateralNoiseStddev = 0.0;
constexpr double kDefaultThetaNoiseStddev = 0.0;
constexpr int kDefaultSeed = 0;

[[nodiscard]] inline auto odometryDefaultConfig() -> OdometrySensorConfig {
  return OdometrySensorConfig{.forwardNoiseStddev = kDefaultForwardNoiseStddev,
                              .lateralNoiseStddev = kDefaultLateralNoiseStddev,
                              .thetaNoiseStddev = kDefaultThetaNoiseStddev,
                              .seed = kDefaultSeed};
}

} // namespace config

class IOdometrySensor {
public:
  virtual ~IOdometrySensor() = default;
  IOdometrySensor() = default;
  IOdometrySensor(const IOdometrySensor &) = delete;
  auto operator=(const IOdometrySensor &) -> IOdometrySensor & = delete;
  IOdometrySensor(IOdometrySensor &&) = delete;
  auto operator=(IOdometrySensor &&) -> IOdometrySensor & = delete;
  [[nodiscard]] virtual auto measure(const types::Pose &previousPose,
                                     const types::Pose &currentPose) const
      -> Result<types::OdometryDelta> = 0;
};

class OdometrySensor final : public IOdometrySensor {
public:
  explicit OdometrySensor(OdometrySensorConfig config = config::odometryDefaultConfig());

  [[nodiscard]] auto measure(const types::Pose &previousPose, const types::Pose &currentPose) const
      -> Result<types::OdometryDelta> override;

private:
  const OdometrySensorConfig config_;
  mutable std::mt19937 generator_;
};

} // namespace ad::simulation

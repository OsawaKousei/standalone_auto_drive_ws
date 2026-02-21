#include "controller_factory.hpp"

#include "pid.hpp"
#include "pure_pursuit.hpp"

namespace ad::control {

namespace {

[[nodiscard]] auto requiredRaw(const ::ad::config::TextConfig &cfg, std::string_view key)
    -> Result<std::string_view> {
  const auto raw = cfg.findRaw("", key);
  if (!raw) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Required control config key is missing: " + std::string{key}});
  }
  return *raw;
}

[[nodiscard]] auto parsePurePursuitConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<PurePursuitConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Control config is required for pure_pursuit."});
  }

  const auto &cfg = *configDoc;
  const auto lookaheadRaw = requiredRaw(cfg, "lookahead_distance");
  if (!lookaheadRaw) {
    return tl::make_unexpected(lookaheadRaw.error());
  }
  const auto lookaheadDistanceValue = ::ad::config::parseDoubleValue(*lookaheadRaw);
  if (!lookaheadDistanceValue) {
    return tl::make_unexpected(lookaheadDistanceValue.error());
  }

  const auto velocityRaw = requiredRaw(cfg, "desired_linear_velocity");
  if (!velocityRaw) {
    return tl::make_unexpected(velocityRaw.error());
  }
  const auto desiredLinearVelocityValue = ::ad::config::parseDoubleValue(*velocityRaw);
  if (!desiredLinearVelocityValue) {
    return tl::make_unexpected(desiredLinearVelocityValue.error());
  }

  return PurePursuitConfig{.lookaheadDistance = *lookaheadDistanceValue,
                           .desiredLinearVelocity = *desiredLinearVelocityValue};
}

[[nodiscard]] auto parsePidConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<PidConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Control config is required for pid."});
  }

  const auto &cfg = *configDoc;

  const auto lookaheadDistanceRaw = requiredRaw(cfg, "lookahead_distance");
  if (!lookaheadDistanceRaw) {
    return tl::make_unexpected(lookaheadDistanceRaw.error());
  }
  const auto lookaheadDistanceValue = ::ad::config::parseDoubleValue(*lookaheadDistanceRaw);
  if (!lookaheadDistanceValue) {
    return tl::make_unexpected(lookaheadDistanceValue.error());
  }

  const auto maxLinearSpeedRaw = requiredRaw(cfg, "max_linear_speed");
  if (!maxLinearSpeedRaw) {
    return tl::make_unexpected(maxLinearSpeedRaw.error());
  }
  const auto maxLinearSpeedValue = ::ad::config::parseDoubleValue(*maxLinearSpeedRaw);
  if (!maxLinearSpeedValue) {
    return tl::make_unexpected(maxLinearSpeedValue.error());
  }

  const auto maxAngularSpeedRaw = requiredRaw(cfg, "max_angular_speed");
  if (!maxAngularSpeedRaw) {
    return tl::make_unexpected(maxAngularSpeedRaw.error());
  }
  const auto maxAngularSpeedValue = ::ad::config::parseDoubleValue(*maxAngularSpeedRaw);
  if (!maxAngularSpeedValue) {
    return tl::make_unexpected(maxAngularSpeedValue.error());
  }

  const auto positionKpRaw = requiredRaw(cfg, "position_kp");
  if (!positionKpRaw) {
    return tl::make_unexpected(positionKpRaw.error());
  }
  const auto positionKpValue = ::ad::config::parseDoubleValue(*positionKpRaw);
  if (!positionKpValue) {
    return tl::make_unexpected(positionKpValue.error());
  }

  const auto positionKiRaw = requiredRaw(cfg, "position_ki");
  if (!positionKiRaw) {
    return tl::make_unexpected(positionKiRaw.error());
  }
  const auto positionKiValue = ::ad::config::parseDoubleValue(*positionKiRaw);
  if (!positionKiValue) {
    return tl::make_unexpected(positionKiValue.error());
  }

  const auto positionKdRaw = requiredRaw(cfg, "position_kd");
  if (!positionKdRaw) {
    return tl::make_unexpected(positionKdRaw.error());
  }
  const auto positionKdValue = ::ad::config::parseDoubleValue(*positionKdRaw);
  if (!positionKdValue) {
    return tl::make_unexpected(positionKdValue.error());
  }

  const auto headingKpRaw = requiredRaw(cfg, "heading_kp");
  if (!headingKpRaw) {
    return tl::make_unexpected(headingKpRaw.error());
  }
  const auto headingKpValue = ::ad::config::parseDoubleValue(*headingKpRaw);
  if (!headingKpValue) {
    return tl::make_unexpected(headingKpValue.error());
  }

  const auto headingKiRaw = requiredRaw(cfg, "heading_ki");
  if (!headingKiRaw) {
    return tl::make_unexpected(headingKiRaw.error());
  }
  const auto headingKiValue = ::ad::config::parseDoubleValue(*headingKiRaw);
  if (!headingKiValue) {
    return tl::make_unexpected(headingKiValue.error());
  }

  const auto headingKdRaw = requiredRaw(cfg, "heading_kd");
  if (!headingKdRaw) {
    return tl::make_unexpected(headingKdRaw.error());
  }
  const auto headingKdValue = ::ad::config::parseDoubleValue(*headingKdRaw);
  if (!headingKdValue) {
    return tl::make_unexpected(headingKdValue.error());
  }

  return PidConfig{.lookaheadDistance = *lookaheadDistanceValue,
                   .maxLinearSpeed = *maxLinearSpeedValue,
                   .maxAngularSpeed = *maxAngularSpeedValue,
                   .positionKp = *positionKpValue,
                   .positionKi = *positionKiValue,
                   .positionKd = *positionKdValue,
                   .headingKp = *headingKpValue,
                   .headingKi = *headingKiValue,
                   .headingKd = *headingKdValue};
}

} // namespace

auto createControllerFromConfig(std::string_view algorithm,
                                const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IController>> {
  if (algorithm == "pure_pursuit") {
    const auto configValue = parsePurePursuitConfig(configDoc);
    if (!configValue) {
      return tl::make_unexpected(configValue.error());
    }

    return std::make_unique<PurePursuitController>(*configValue);
  }

  if (algorithm == "pid") {
    const auto configValue = parsePidConfig(configDoc);
    if (!configValue) {
      return tl::make_unexpected(configValue.error());
    }

    return std::make_unique<PidController>(*configValue);
  }

  return tl::make_unexpected(
      Error{.code = ErrorCode::InvalidInput,
            .message = "Unsupported control algorithm: " + std::string{algorithm}});
}

} // namespace ad::control

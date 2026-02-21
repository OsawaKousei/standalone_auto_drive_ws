#include "controller_factory.hpp"

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

} // namespace

auto createControllerFromConfig(std::string_view algorithm,
                                const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IController>> {
  if (algorithm != "pure_pursuit") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported control algorithm: " + std::string{algorithm}});
  }

  const auto configValue = parsePurePursuitConfig(configDoc);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  return std::make_unique<PurePursuitController>(*configValue);
}

} // namespace ad::control

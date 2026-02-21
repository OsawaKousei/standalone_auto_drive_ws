#include "controller_factory.hpp"

#include "pure_pursuit.hpp"

namespace ad::control {

namespace {

[[nodiscard]] auto parsePurePursuitConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<PurePursuitConfig> {
  auto lookaheadDistanceValue = config::kDefaultLookaheadDistance;
  auto desiredLinearVelocityValue = config::kDefaultDesiredLinearVelocity;
  if (!configDoc.has_value()) {
    return config::purePursuitDefaultConfig();
  }

  const auto &cfg = *configDoc;
  const auto lookaheadRaw = cfg.findRaw("", "lookahead_distance");
  if (lookaheadRaw) {
    const auto parsed = ::ad::config::parseDoubleValue(*lookaheadRaw);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    lookaheadDistanceValue = *parsed;
  }

  const auto velocityRaw = cfg.findRaw("", "desired_linear_velocity");
  if (velocityRaw) {
    const auto parsed = ::ad::config::parseDoubleValue(*velocityRaw);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    desiredLinearVelocityValue = *parsed;
  }

  return PurePursuitConfig{.lookaheadDistance = lookaheadDistanceValue,
                           .desiredLinearVelocity = desiredLinearVelocityValue};
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

  return std::unique_ptr<IController>{new PurePursuitController{*configValue}};
}

} // namespace ad::control

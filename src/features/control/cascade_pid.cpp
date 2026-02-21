#include "cascade_pid.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ranges>

namespace {

constexpr auto kAnglePeriod = 2.0 * std::numbers::pi;

[[nodiscard]] auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, kAnglePeriod);
  if (angle < 0.0) {
    angle += kAnglePeriod;
  }
  return angle - std::numbers::pi;
}

} // namespace

namespace ad::control {

CascadePidController::CascadePidController(CascadePidConfig config) : config_(config) {}

auto CascadePidController::selectLookaheadTarget(std::span<const types::Point> path,
                                                 const types::Pose &pose, double lookaheadDistance)
    -> types::Point {
  if (path.size() == 1U) {
    return path.front();
  }

  const auto indices = std::views::iota(std::size_t{0}, path.size());
  const auto closestIndex =
      *std::ranges::min_element(indices, [&](std::size_t lhs, std::size_t rhs) -> bool {
        const auto dxLeft = path[lhs].x - pose.x;
        const auto dyLeft = path[lhs].y - pose.y;
        const auto dxRight = path[rhs].x - pose.x;
        const auto dyRight = path[rhs].y - pose.y;
        return std::hypot(dxLeft, dyLeft) < std::hypot(dxRight, dyRight);
      });

  auto remaining = lookaheadDistance;
  auto currentX = path[closestIndex].x;
  auto currentY = path[closestIndex].y;
  const auto lastIndex = path.size() - 1U;
  for (const auto index : std::views::iota(closestIndex, lastIndex)) {
    const auto next = path[index + 1U];
    const auto segment = std::hypot(next.x - currentX, next.y - currentY);
    if (segment <= 0.0) {
      currentX = next.x;
      currentY = next.y;
      continue;
    }
    if (remaining <= segment) {
      const auto ratio = remaining / segment;
      return types::Point{.x = currentX + (ratio * (next.x - currentX)),
                          .y = currentY + (ratio * (next.y - currentY))};
    }
    remaining -= segment;
    currentX = next.x;
    currentY = next.y;
  }

  return path.back();
}

auto CascadePidController::updatePid(PidState &state, double error, double deltaSeconds,
                                     const PidGains &gains, double integralLimit) -> double {
  state.integral += error * deltaSeconds;
  state.integral = std::clamp(state.integral, -integralLimit, integralLimit);

  auto derivative = 0.0;
  if (state.initialized) {
    derivative = (error - state.previousError) / deltaSeconds;
  }

  state.previousError = error;
  state.initialized = true;

  return (gains.proportional * error) + (gains.integral * state.integral) +
         (gains.derivative * derivative);
}

auto CascadePidController::computeCommand(const ControlInput &input) const -> Result<types::Twist> {
  if (input.path.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::EmptyCollection, .message = "Path is empty."});
  }

  if (input.deltaSeconds <= 0.0 || !std::isfinite(input.deltaSeconds)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Control delta time must be positive."});
  }

  if (config_.lookaheadDistance <= 0.0 || config_.maxLinearSpeed <= 0.0 ||
      config_.maxAngularSpeed <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Cascade PID limits must be positive."});
  }

  const auto target =
      selectLookaheadTarget(input.path, input.currentPose, config_.lookaheadDistance);

  auto measuredBodyVx = 0.0;
  auto measuredBodyVy = 0.0;
  auto measuredYawRate = 0.0;

  if (previousPose_.has_value()) {
    const auto deltaPosX = input.currentPose.x - previousPose_->x;
    const auto deltaPosY = input.currentPose.y - previousPose_->y;
    const auto dTheta = normalizeAngle(input.currentPose.theta - previousPose_->theta);

    const auto worldVx = deltaPosX / input.deltaSeconds;
    const auto worldVy = deltaPosY / input.deltaSeconds;

    const auto cosTheta = std::cos(input.currentPose.theta);
    const auto sinTheta = std::sin(input.currentPose.theta);
    measuredBodyVx = (cosTheta * worldVx) + (sinTheta * worldVy);
    measuredBodyVy = (-sinTheta * worldVx) + (cosTheta * worldVy);
    measuredYawRate = dTheta / input.deltaSeconds;
  }

  const auto errorX = target.x - input.currentPose.x;
  const auto errorY = target.y - input.currentPose.y;

  const auto positionGains = PidGains{.proportional = config_.positionKp,
                                      .integral = config_.positionKi,
                                      .derivative = config_.positionKd};
  const auto velocityGains = PidGains{.proportional = config_.velocityKp,
                                      .integral = config_.velocityKi,
                                      .derivative = config_.velocityKd};
  const auto headingGains = PidGains{.proportional = config_.headingKp,
                                     .integral = config_.headingKi,
                                     .derivative = config_.headingKd};

  const auto desiredWorldVx =
      updatePid(positionXState_, errorX, input.deltaSeconds, positionGains, config_.maxLinearSpeed);
  const auto desiredWorldVy =
      updatePid(positionYState_, errorY, input.deltaSeconds, positionGains, config_.maxLinearSpeed);

  const auto cosTheta = std::cos(input.currentPose.theta);
  const auto sinTheta = std::sin(input.currentPose.theta);
  const auto desiredBodyVx = std::clamp((cosTheta * desiredWorldVx) + (sinTheta * desiredWorldVy),
                                        -config_.maxLinearSpeed, config_.maxLinearSpeed);
  const auto desiredBodyVy = std::clamp((-sinTheta * desiredWorldVx) + (cosTheta * desiredWorldVy),
                                        -config_.maxLinearSpeed, config_.maxLinearSpeed);

  const auto targetHeading = std::atan2(errorY, errorX);
  const auto headingError = normalizeAngle(targetHeading - input.currentPose.theta);
  const auto desiredYawRate = std::clamp(updatePid(headingState_, headingError, input.deltaSeconds,
                                                   headingGains, config_.maxAngularSpeed),
                                         -config_.maxAngularSpeed, config_.maxAngularSpeed);

  const auto velocityErrorX = desiredBodyVx - measuredBodyVx;
  const auto velocityErrorY = desiredBodyVy - measuredBodyVy;
  const auto yawRateError = desiredYawRate - measuredYawRate;

  const auto velocityCorrectionX = updatePid(velocityXState_, velocityErrorX, input.deltaSeconds,
                                             velocityGains, config_.maxLinearSpeed);
  const auto velocityCorrectionY = updatePid(velocityYState_, velocityErrorY, input.deltaSeconds,
                                             velocityGains, config_.maxLinearSpeed);
  const auto yawRateCorrection = updatePid(yawRateState_, yawRateError, input.deltaSeconds,
                                           velocityGains, config_.maxAngularSpeed);

  previousPose_.emplace(input.currentPose);

  return types::Twist{.v = std::clamp(desiredBodyVx + velocityCorrectionX, -config_.maxLinearSpeed,
                                      config_.maxLinearSpeed),
                      .vy = std::clamp(desiredBodyVy + velocityCorrectionY, -config_.maxLinearSpeed,
                                       config_.maxLinearSpeed),
                      .w = std::clamp(desiredYawRate + yawRateCorrection, -config_.maxAngularSpeed,
                                      config_.maxAngularSpeed)};
}

} // namespace ad::control

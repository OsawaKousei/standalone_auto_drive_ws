#include "pid.hpp"

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

PidController::PidController(PidConfig config) : config_(config) {}

auto PidController::selectLookaheadTarget(std::span<const types::Point> path,
                                          const types::Pose &pose, double lookaheadDistance,
                                          std::size_t minClosestIndex) -> LookaheadSelection {
  if (minClosestIndex >= path.size()) {
    minClosestIndex = path.size() - 1U;
  }

  if (path.size() == 1U) {
    return LookaheadSelection{.target = path.front(), .closestIndex = 0U};
  }

  const auto indices = std::views::iota(minClosestIndex, path.size());
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
      return LookaheadSelection{.target =
                                    types::Point{.x = currentX + (ratio * (next.x - currentX)),
                                                 .y = currentY + (ratio * (next.y - currentY))},
                                .closestIndex = closestIndex};
    }
    remaining -= segment;
    currentX = next.x;
    currentY = next.y;
  }

  return LookaheadSelection{.target = path.back(), .closestIndex = closestIndex};
}

auto PidController::updatePid(PidState &state, double error, double deltaSeconds,
                              const PidGains &gains, double integralLimit) -> double {
  state.integral += error * deltaSeconds;
  state.integral = std::clamp(state.integral, -integralLimit, integralLimit);

  auto derivative = 0.0;
  if (state.initialized) {
    derivative = (error - state.previousError) / deltaSeconds;
  }

  state.previousError = error;
  state.initialized = true;

  const auto proportionalTerm = gains.proportional * error;
  auto derivativeTerm = gains.derivative * derivative;
  const auto derivativeLimit = std::abs(proportionalTerm);
  derivativeTerm = std::clamp(derivativeTerm, -derivativeLimit, derivativeLimit);

  return proportionalTerm + (gains.integral * state.integral) + derivativeTerm;
}

auto PidController::computeCommand(const ControlInput &input) const -> Result<types::Twist> {
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
        Error{.code = ErrorCode::InvalidInput, .message = "PID limits must be positive."});
  }

  if (previousPathSize_ != input.path.size()) {
    previousPathSize_ = input.path.size();
    pathProgressIndex_ = 0U;
  }

  const auto minClosestIndex = std::min(pathProgressIndex_, input.path.size() - 1U);
  const auto lookahead = selectLookaheadTarget(input.path, input.currentPose,
                                               config_.lookaheadDistance, minClosestIndex);
  pathProgressIndex_ = std::max(pathProgressIndex_, lookahead.closestIndex);
  const auto target = lookahead.target;

  const auto errorX = target.x - input.currentPose.x;
  const auto errorY = target.y - input.currentPose.y;

  const auto positionGains = PidGains{.proportional = config_.positionKp,
                                      .integral = config_.positionKi,
                                      .derivative = config_.positionKd};
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

  auto commandV = desiredBodyVx;
  auto commandVy = desiredBodyVy;
  auto commandW = desiredYawRate;

  if (previousCommand_.has_value()) {
    const auto maxLinearDelta = config_.maxLinearSpeed * input.deltaSeconds;
    const auto maxAngularDelta = config_.maxAngularSpeed * input.deltaSeconds;

    commandV = std::clamp(commandV, previousCommand_->v - maxLinearDelta,
                          previousCommand_->v + maxLinearDelta);
    commandVy = std::clamp(commandVy, previousCommand_->vy - maxLinearDelta,
                           previousCommand_->vy + maxLinearDelta);
    commandW = std::clamp(commandW, previousCommand_->w - maxAngularDelta,
                          previousCommand_->w + maxAngularDelta);
  }

  const auto command = types::Twist{.v = commandV, .vy = commandVy, .w = commandW};

  previousCommand_.emplace(command);

  return command;
}

} // namespace ad::control

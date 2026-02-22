#include "localizer_util.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace ad::localization::util {

namespace {

constexpr double kTwo = 2.0;
constexpr double kTwoPi = kTwo * std::numbers::pi;

} // namespace

auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - std::numbers::pi;
}

} // namespace ad::localization::util

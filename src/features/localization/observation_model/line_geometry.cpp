#include "line_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace {

constexpr double kEpsilon = 1e-9;

} // namespace

namespace ad::localization::line_geometry {

auto buildMapLineFromSegment(const types::Point &start, const types::Point &end,
                             const double minSegmentLength) -> std::optional<util::MapLine> {
  const auto dx = end.x - start.x;
  const auto dy = end.y - start.y;
  const auto length = std::hypot(dx, dy);
  if (length < std::max(kEpsilon, minSegmentLength)) {
    return std::nullopt;
  }

  const auto directionX = dx / length;
  const auto directionY = dy / length;
  const auto alphaRaw = std::atan2(dy, dx) + (0.5 * std::numbers::pi);
  const auto normalX = std::cos(alphaRaw);
  const auto normalY = std::sin(alphaRaw);
  const auto rhoRaw = (normalX * start.x) + (normalY * start.y);
  const auto model = util::toLineModel(util::LineModel{.rho = rhoRaw, .alpha = alphaRaw});

  auto minProjection = (directionX * start.x) + (directionY * start.y);
  auto maxProjection = (directionX * end.x) + (directionY * end.y);
  if (minProjection > maxProjection) {
    std::swap(minProjection, maxProjection);
  }

  return util::MapLine{types::LineSegment{.start = start, .end = end},
                       model,
                       directionX,
                       directionY,
                       minProjection,
                       maxProjection};
}

auto buildMapLineFromModelAndProjectionSpan(const util::LineModel &model, double minProjection,
                                            double maxProjection, const double minSegmentLength)
    -> std::optional<util::MapLine> {
  if (minProjection > maxProjection) {
    std::swap(minProjection, maxProjection);
  }

  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);
  const auto tangentX = -normalY;
  const auto tangentY = normalX;

  const auto start = types::Point{.x = (tangentX * minProjection) + (normalX * model.rho),
                                  .y = (tangentY * minProjection) + (normalY * model.rho)};
  const auto end = types::Point{.x = (tangentX * maxProjection) + (normalX * model.rho),
                                .y = (tangentY * maxProjection) + (normalY * model.rho)};

  return buildMapLineFromSegment(start, end, minSegmentLength);
}

auto transformLineModelLocalToMap(const util::LineModel &localLine, const types::Pose &pose)
    -> util::LineModel {
  const auto alphaMap = util::normalizeAngle(localLine.alpha + pose.theta);
  const auto rhoMap = localLine.rho + (pose.x * std::cos(alphaMap)) + (pose.y * std::sin(alphaMap));
  return util::toLineModel(util::LineModel{.rho = rhoMap, .alpha = alphaMap});
}

} // namespace ad::localization::line_geometry

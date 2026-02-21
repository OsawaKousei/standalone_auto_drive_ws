#include "hough_ransac_core.hpp"

#include <cmath>
#include <random>

namespace {

constexpr double kEpsilon = 1e-9;

auto lineFromTwoPoints(const ad::types::Point &point1, const ad::types::Point &point2)
    -> std::optional<ad::localization::util::LineModel> {
  const auto deltaX = point2.x - point1.x;
  const auto deltaY = point2.y - point1.y;
  const auto norm = std::hypot(deltaX, deltaY);
  if (norm < kEpsilon) {
    return std::nullopt;
  }

  const auto normalX = -deltaY / norm;
  const auto normalY = deltaX / norm;
  const auto rho = (normalX * point1.x) + (normalY * point1.y);
  const auto alpha = std::atan2(normalY, normalX);
  return ad::localization::util::toLineModel(
      ad::localization::util::LineModel{.rho = rho, .alpha = alpha});
}

auto collectInliers(const std::vector<ad::types::Point> &points,
                    const ad::localization::util::LineModel &model, double inlierDistance)
    -> std::vector<ad::types::Point> {
  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);

  auto inliers = std::vector<ad::types::Point>{};
  inliers.reserve(points.size());
  for (const auto &point : points) {
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - model.rho);
    if (distance <= inlierDistance) {
      inliers.push_back(point);
    }
  }

  return inliers;
}

} // namespace

namespace ad::localization::ransac {

auto fitLineToPoints(const std::vector<types::Point> &points, const RansacConfig &config,
                     std::uint32_t randomSeed) -> std::optional<RansacLineFitResult> {
  if (points.size() < 2U || config.maxIterations <= 0 || !(config.inlierDistance > 0.0) ||
      config.minInliers < 2U || config.minInlierRatio <= 0.0 || config.minInlierRatio > 1.0) {
    return std::nullopt;
  }

  const auto seed =
      randomSeed == 0U ? static_cast<std::uint32_t>(points.size() * 2654435761U) : randomSeed;
  std::mt19937 generator(seed);
  std::uniform_int_distribution<std::size_t> distribution(0U, points.size() - 1U);

  auto bestFit = std::optional<RansacLineFitResult>{};
  for (int iteration = 0; iteration < config.maxIterations; ++iteration) {
    const auto index1 = distribution(generator);
    const auto index2 = distribution(generator);
    if (index1 == index2) {
      continue;
    }

    const auto model = lineFromTwoPoints(points[index1], points[index2]);
    if (!model) {
      continue;
    }

    const auto inliers = collectInliers(points, *model, config.inlierDistance);
    if (inliers.size() < config.minInliers) {
      continue;
    }

    const auto inlierRatio =
        static_cast<double>(inliers.size()) / static_cast<double>(points.size());
    if (inlierRatio < config.minInlierRatio) {
      continue;
    }

    const auto fit = util::fitLine(inliers);
    if (!fit) {
      continue;
    }

    const auto candidate = RansacLineFitResult{.fit = *fit, .inlierCount = inliers.size()};
    if (!bestFit || candidate.inlierCount > bestFit->inlierCount ||
        (candidate.inlierCount == bestFit->inlierCount && candidate.fit.mse < bestFit->fit.mse)) {
      bestFit = candidate;
    }
  }

  return bestFit;
}

} // namespace ad::localization::ransac

#include "ransac_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

auto collectInlierIndices(const std::vector<ad::types::Point> &points,
                          const ad::localization::util::LineModel &model, double inlierDistance)
    -> std::vector<std::size_t> {
  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);

  auto inlierIndices = std::vector<std::size_t>{};
  inlierIndices.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto &point = points[index];
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - model.rho);
    if (distance <= inlierDistance) {
      inlierIndices.push_back(index);
    }
  }

  return inlierIndices;
}

auto computeProjectionSpan(const std::vector<ad::types::Point> &points,
                           const std::vector<std::size_t> &indices,
                           const ad::localization::util::LineModel &model) -> double {
  if (indices.empty()) {
    return 0.0;
  }

  const auto tangentX = -std::sin(model.alpha);
  const auto tangentY = std::cos(model.alpha);

  auto minProjection = std::numeric_limits<double>::infinity();
  auto maxProjection = -std::numeric_limits<double>::infinity();
  for (const auto index : indices) {
    const auto &point = points[index];
    const auto projection = (tangentX * point.x) + (tangentY * point.y);
    minProjection = std::min(minProjection, projection);
    maxProjection = std::max(maxProjection, projection);
  }

  if (!std::isfinite(minProjection) || !std::isfinite(maxProjection)) {
    return 0.0;
  }
  return std::max(0.0, maxProjection - minProjection);
}

auto drawUniqueSample(std::mt19937 &generator, std::size_t pointCount, std::size_t sampleSize)
    -> std::vector<std::size_t> {
  auto sample = std::vector<std::size_t>{};
  sample.reserve(sampleSize);
  std::uniform_int_distribution<std::size_t> distribution(0U, pointCount - 1U);

  while (sample.size() < sampleSize) {
    const auto candidate = distribution(generator);
    const auto exists = std::find(sample.begin(), sample.end(), candidate) != sample.end();
    if (!exists) {
      sample.push_back(candidate);
    }
  }

  return sample;
}

} // namespace

namespace ad::localization::ransac {

auto runGenericPointRansac(const std::vector<types::Point> &points,
                           const GenericRansacConfig &config, const InlierIndexSelector &selector,
                           std::uint32_t randomSeed) -> std::optional<GenericRansacResult> {
  if (points.size() < config.sampleSize || config.maxIterations <= 0 || config.sampleSize < 2U ||
      config.minInliers < config.sampleSize || config.minInlierRatio <= 0.0 ||
      config.minInlierRatio > 1.0 || !selector) {
    return std::nullopt;
  }

  const auto seed =
      randomSeed == 0U ? static_cast<std::uint32_t>(points.size() * 2654435761U) : randomSeed;
  std::mt19937 generator(seed);

  auto bestResult = std::optional<GenericRansacResult>{};
  for (int iteration = 0; iteration < config.maxIterations; ++iteration) {
    const auto sampleIndices = drawUniqueSample(generator, points.size(), config.sampleSize);
    const auto inlierIndices = selector(points, sampleIndices);
    if (!inlierIndices) {
      continue;
    }

    if (inlierIndices->size() < config.minInliers) {
      continue;
    }

    const auto inlierRatio =
        static_cast<double>(inlierIndices->size()) / static_cast<double>(points.size());
    if (inlierRatio < config.minInlierRatio) {
      continue;
    }

    if (!bestResult || inlierIndices->size() > bestResult->inlierIndices.size()) {
      bestResult = GenericRansacResult{.inlierIndices = *inlierIndices};
    }
  }

  return bestResult;
}

auto fitLineToPoints(const std::vector<types::Point> &points, const RansacConfig &config,
                     std::uint32_t randomSeed) -> std::optional<RansacLineFitResult> {
  if (points.size() < 2U || config.maxIterations <= 0 || !(config.inlierDistance > 0.0) ||
      config.minInliers < 2U || config.minInlierRatio <= 0.0 || config.minInlierRatio > 1.0 ||
      config.minInlierSpan <= 0.0) {
    return std::nullopt;
  }

  const auto genericResult = runGenericPointRansac(
      points,
      GenericRansacConfig{.maxIterations = config.maxIterations,
                          .sampleSize = 2U,
                          .minInliers = config.minInliers,
                          .minInlierRatio = config.minInlierRatio},
      [&](const std::vector<types::Point> &allPoints, const std::vector<std::size_t> &sampleIndices)
          -> std::optional<std::vector<std::size_t>> {
        if (sampleIndices.size() != 2U) {
          return std::nullopt;
        }
        const auto model =
            lineFromTwoPoints(allPoints[sampleIndices[0]], allPoints[sampleIndices[1]]);
        if (!model) {
          return std::nullopt;
        }
        auto inlierIndices = collectInlierIndices(allPoints, *model, config.inlierDistance);
        const auto inlierSpan = computeProjectionSpan(allPoints, inlierIndices, *model);
        if (inlierSpan < config.minInlierSpan) {
          return std::nullopt;
        }
        return inlierIndices;
      },
      randomSeed);

  if (!genericResult) {
    return std::nullopt;
  }

  auto bestFit = std::optional<RansacLineFitResult>{};
  const auto &indices = genericResult->inlierIndices;
  if (indices.empty()) {
    return std::nullopt;
  }

  auto inliers = std::vector<types::Point>{};
  inliers.reserve(indices.size());
  for (const auto index : indices) {
    inliers.push_back(points[index]);
  }

  const auto fit = util::fitLine(inliers);
  if (!fit) {
    return std::nullopt;
  }

  const auto inlierSpan = computeProjectionSpan(points, indices, fit->model);
  bestFit = RansacLineFitResult{.fit = *fit,
                                .inlierCount = inliers.size(),
                                .inlierSpan = inlierSpan,
                                .inlierIndices = indices};
  return bestFit;
}

} // namespace ad::localization::ransac

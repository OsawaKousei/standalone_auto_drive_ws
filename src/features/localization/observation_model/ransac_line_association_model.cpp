#include "ransac_line_association_model.hpp"

#include "../localizer_util.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;
constexpr std::uint32_t kPointRansacSeedBias = 17U;
constexpr std::uint32_t kAssociationRansacSeedBias = 97U;
constexpr int kDistinctSampleRetryCount = 8;
constexpr double kClusterSplitDistanceMultiplier = 3.0;

struct ObservedLine {
  ad::localization::observation_model::util::LineModel model;
  ad::types::LineSegment segment;
  std::size_t supportPointCount;
  double mse;
};

struct ExtractedLineCandidate {
  std::vector<std::size_t> inlierIndices;
  std::optional<ObservedLine> observedLine;
};

struct ScanPoint {
  ad::types::Point point;
  std::size_t scanIndex;
};

struct LocalPointFeature {
  double directionX;
  double directionY;
  double linearity;
};

struct PoseMatchResult {
  ad::types::Pose pose;
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  double mahalanobisDistanceSquared;
};

auto mixToUint32(const std::uint64_t value) -> std::uint32_t {
  auto mixed = value;
  mixed ^= mixed >> 33U;
  mixed *= 0xff51afd7ed558ccdULL;
  mixed ^= mixed >> 33U;
  mixed *= 0xc4ceb9fe1a85ec53ULL;
  mixed ^= mixed >> 33U;
  return static_cast<std::uint32_t>(mixed & 0xffffffffULL);
}

auto buildAssociationRansacSeed(const std::size_t observedCount, const std::size_t mapCount,
                                const ad::types::Pose &predictedPose) -> std::uint32_t {
  auto randomDevice = std::random_device{};
  const auto now =
      static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto quantizedX = static_cast<std::int64_t>(std::llround(predictedPose.x * 1000.0));
  const auto quantizedY = static_cast<std::int64_t>(std::llround(predictedPose.y * 1000.0));
  const auto quantizedTheta =
      static_cast<std::int64_t>(std::llround(predictedPose.theta * 1000000.0));

  auto seedSequence = std::seed_seq{randomDevice(),
                                    randomDevice(),
                                    randomDevice(),
                                    static_cast<std::uint32_t>(observedCount),
                                    static_cast<std::uint32_t>(mapCount),
                                    mixToUint32(now),
                                    mixToUint32(static_cast<std::uint64_t>(quantizedX)),
                                    mixToUint32(static_cast<std::uint64_t>(quantizedY)),
                                    mixToUint32(static_cast<std::uint64_t>(quantizedTheta)),
                                    kAssociationRansacSeedBias};
  auto seed = std::uint32_t{0U};
  seedSequence.generate(&seed, &seed + 1);
  return seed;
}

auto sampleDistinctIndices(std::mt19937 &rng, const std::size_t size)
    -> std::optional<std::pair<std::size_t, std::size_t>> {
  if (size < 2U) {
    return std::nullopt;
  }

  auto distribution = std::uniform_int_distribution<std::size_t>{0, size - 1U};
  const auto firstIndex = distribution(rng);
  auto secondIndex = distribution(rng);
  for (int retries = 0; retries < kDistinctSampleRetryCount && secondIndex == firstIndex;
       ++retries) {
    secondIndex = distribution(rng);
  }

  if (secondIndex == firstIndex) {
    return std::nullopt;
  }
  return std::pair<std::size_t, std::size_t>{firstIndex, secondIndex};
}

auto transformPointToMap(const ad::types::Point &point, const ad::types::Pose &pose)
    -> ad::types::Point {
  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);
  return ad::types::Point{.x = pose.x + (cosTheta * point.x) - (sinTheta * point.y),
                          .y = pose.y + (sinTheta * point.x) + (cosTheta * point.y)};
}

auto transformLineToMap(const ad::localization::observation_model::util::LineModel &line,
                        const ad::types::Pose &pose)
    -> ad::localization::observation_model::util::LineModel {
  const auto alphaMap = ad::localization::util::normalizeAngle(line.alpha + pose.theta);
  const auto normalX = std::cos(alphaMap);
  const auto normalY = std::sin(alphaMap);
  const auto rhoMap = line.rho + (normalX * pose.x) + (normalY * pose.y);
  return ad::localization::observation_model::util::toLineModel(
      ad::localization::observation_model::util::LineModel{.rho = rhoMap, .alpha = alphaMap});
}

auto buildScanPoints(const ad::types::LidarScan &scan) -> std::vector<ScanPoint> {
  auto points = std::vector<ScanPoint>{};
  points.reserve(scan.ranges.size());

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }

    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    points.push_back(ScanPoint{
        .point = ad::types::Point{.x = range * std::cos(angle), .y = range * std::sin(angle)},
        .scanIndex = index});
  }

  return points;
}

auto buildLocalPointFeatures(const std::vector<ScanPoint> &points,
                             const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<LocalPointFeature> {
  auto features = std::vector<LocalPointFeature>(
      points.size(), LocalPointFeature{.directionX = 1.0, .directionY = 0.0, .linearity = 0.0});
  if (points.size() < 2U) {
    return features;
  }

  const auto windowRadius = static_cast<std::size_t>(std::max(1, config.localPcaWindowSize));
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto start = index > windowRadius ? index - windowRadius : 0U;
    const auto end = std::min(points.size() - 1U, index + windowRadius);
    if ((end - start + 1U) < 2U) {
      continue;
    }

    double meanX = 0.0;
    double meanY = 0.0;
    for (std::size_t local = start; local <= end; ++local) {
      meanX += points[local].point.x;
      meanY += points[local].point.y;
    }
    const auto count = static_cast<double>(end - start + 1U);
    meanX /= count;
    meanY /= count;

    double sxx = 0.0;
    double sxy = 0.0;
    double syy = 0.0;
    for (std::size_t local = start; local <= end; ++local) {
      const auto deltaX = points[local].point.x - meanX;
      const auto deltaY = points[local].point.y - meanY;
      sxx += deltaX * deltaX;
      sxy += deltaX * deltaY;
      syy += deltaY * deltaY;
    }

    const auto trace = sxx + syy;
    if (trace < kEpsilon) {
      continue;
    }

    const auto direction = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    const auto directionX = std::cos(direction);
    const auto directionY = std::sin(direction);
    const auto determinant = (sxx * syy) - (sxy * sxy);
    const auto discriminant = std::max(0.0, (trace * trace) - (4.0 * determinant));
    const auto root = std::sqrt(discriminant);
    const auto lambda1 = 0.5 * (trace + root);
    const auto lambda2 = 0.5 * (trace - root);
    const auto linearity = std::clamp((lambda1 - lambda2) / std::max(kEpsilon, trace), 0.0, 1.0);

    features[index] = LocalPointFeature{
        .directionX = directionX, .directionY = directionY, .linearity = linearity};
  }

  return features;
}

auto fitLineFromPoints(const std::vector<ad::types::Point> &points)
    -> std::optional<ad::localization::observation_model::util::LineModel> {
  if (points.size() < 2U) {
    return std::nullopt;
  }

  double meanX = 0.0;
  double meanY = 0.0;
  for (const auto &point : points) {
    meanX += point.x;
    meanY += point.y;
  }
  const auto count = static_cast<double>(points.size());
  meanX /= count;
  meanY /= count;

  auto momentSums = std::array<double, 3>{0.0, 0.0, 0.0};
  for (const auto &point : points) {
    const auto deltaX = point.x - meanX;
    const auto deltaY = point.y - meanY;
    momentSums[0] += deltaX * deltaX;
    momentSums[1] += deltaX * deltaY;
    momentSums[2] += deltaY * deltaY;
  }

  const auto sxx = momentSums[0];
  const auto sxy = momentSums[1];
  const auto syy = momentSums[2];
  if (sxx + syy < kEpsilon) {
    return std::nullopt;
  }

  const auto direction = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  const auto normal = direction + (0.5 * std::numbers::pi);
  const auto normalX = std::cos(normal);
  const auto normalY = std::sin(normal);
  const auto rho = (normalX * meanX) + (normalY * meanY);

  return ad::localization::observation_model::util::toLineModel(
      ad::localization::observation_model::util::LineModel{.rho = rho, .alpha = normal});
}

auto sampleLineFromPoints(const ad::types::Point &first, const ad::types::Point &second)
    -> std::optional<ad::localization::observation_model::util::LineModel> {
  const auto deltaX = second.x - first.x;
  const auto deltaY = second.y - first.y;
  const auto length = std::hypot(deltaX, deltaY);
  if (length < kEpsilon) {
    return std::nullopt;
  }

  const auto directionX = deltaX / length;
  const auto directionY = deltaY / length;
  const auto normalX = -directionY;
  const auto normalY = directionX;
  const auto alpha = std::atan2(normalY, normalX);
  const auto rho = (normalX * first.x) + (normalY * first.y);
  return ad::localization::observation_model::util::toLineModel(
      ad::localization::observation_model::util::LineModel{.rho = rho, .alpha = alpha});
}

auto sampleLocalPairIndices(std::mt19937 &rng, const std::vector<std::size_t> &activeIndices,
                            const int neighborWindow)
    -> std::optional<std::pair<std::size_t, std::size_t>> {
  if (activeIndices.size() < 2U) {
    return std::nullopt;
  }

  auto fullDistribution = std::uniform_int_distribution<std::size_t>{0, activeIndices.size() - 1U};
  const auto firstPos = fullDistribution(rng);
  const auto window = static_cast<std::size_t>(std::max(1, neighborWindow));
  const auto start = firstPos > window ? firstPos - window : 0U;
  const auto end = std::min(activeIndices.size() - 1U, firstPos + window);
  if (start == end) {
    return std::nullopt;
  }

  auto localDistribution = std::uniform_int_distribution<std::size_t>{start, end};
  auto secondPos = localDistribution(rng);
  for (int retries = 0; retries < kDistinctSampleRetryCount && secondPos == firstPos; ++retries) {
    secondPos = localDistribution(rng);
  }
  if (secondPos == firstPos) {
    return std::nullopt;
  }

  return std::pair<std::size_t, std::size_t>{activeIndices[firstPos], activeIndices[secondPos]};
}

auto collectHybridInlierIndices(const std::vector<ScanPoint> &points,
                                const std::vector<LocalPointFeature> &features,
                                const std::vector<bool> &activeMask,
                                const ad::localization::observation_model::util::LineModel &model,
                                const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<std::size_t> {
  auto inlierCandidates = std::vector<std::size_t>{};
  inlierCandidates.reserve(points.size());
  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);
  const auto lineDirectionX = -std::sin(model.alpha);
  const auto lineDirectionY = std::cos(model.alpha);

  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!activeMask[index]) {
      continue;
    }

    const auto &point = points[index].point;
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - model.rho);
    if (distance > config.pointDistanceThreshold) {
      continue;
    }

    if (features[index].linearity < config.minLinearity) {
      continue;
    }

    const auto alignment = std::abs((lineDirectionX * features[index].directionX) +
                                    (lineDirectionY * features[index].directionY));
    if (alignment < config.minDirectionAlignment) {
      continue;
    }

    inlierCandidates.push_back(index);
  }

  if (inlierCandidates.empty()) {
    return {};
  }

  const auto continuityGap = static_cast<std::size_t>(std::max(1, config.maxContinuityGap));
  const auto maxNeighborDistance =
      std::max(kEpsilon, config.pointDistanceThreshold * kClusterSplitDistanceMultiplier);
  std::size_t bestStart = 0U;
  std::size_t bestLength = 1U;
  std::size_t runStart = 0U;
  std::size_t runLength = 1U;

  for (std::size_t index = 1U; index < inlierCandidates.size(); ++index) {
    const auto previousScanIndex = points[inlierCandidates[index - 1U]].scanIndex;
    const auto currentScanIndex = points[inlierCandidates[index]].scanIndex;
    const auto &previousPoint = points[inlierCandidates[index - 1U]].point;
    const auto &currentPoint = points[inlierCandidates[index]].point;
    const auto euclideanGap =
        std::hypot(currentPoint.x - previousPoint.x, currentPoint.y - previousPoint.y);

    if ((currentScanIndex - previousScanIndex) <= continuityGap &&
        euclideanGap <= maxNeighborDistance) {
      ++runLength;
    } else {
      if (runLength > bestLength) {
        bestLength = runLength;
        bestStart = runStart;
      }
      runStart = index;
      runLength = 1U;
    }
  }
  if (runLength > bestLength) {
    bestLength = runLength;
    bestStart = runStart;
  }

  auto contiguousInliers = std::vector<std::size_t>{};
  contiguousInliers.reserve(bestLength);
  for (std::size_t index = bestStart; index < (bestStart + bestLength); ++index) {
    contiguousInliers.push_back(inlierCandidates[index]);
  }

  return contiguousInliers;
}

auto buildObservedLine(const std::vector<ad::types::Point> &supportPoints,
                       const ad::localization::observation_model::util::LineModel &model,
                       const double minSegmentLength) -> std::optional<ObservedLine> {
  if (supportPoints.size() < 2U) {
    return std::nullopt;
  }

  const auto directionX = -std::sin(model.alpha);
  const auto directionY = std::cos(model.alpha);
  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);
  const auto baseX = normalX * model.rho;
  const auto baseY = normalY * model.rho;

  auto minProjection = std::numeric_limits<double>::infinity();
  auto maxProjection = -std::numeric_limits<double>::infinity();
  double mse = 0.0;
  for (const auto &point : supportPoints) {
    const auto projection = (directionX * point.x) + (directionY * point.y);
    minProjection = std::min(minProjection, projection);
    maxProjection = std::max(maxProjection, projection);

    const auto distance = (normalX * point.x) + (normalY * point.y) - model.rho;
    mse += distance * distance;
  }
  mse /= static_cast<double>(supportPoints.size());

  if (!std::isfinite(minProjection) || !std::isfinite(maxProjection) ||
      (maxProjection - minProjection) < minSegmentLength) {
    return std::nullopt;
  }

  const auto start = ad::types::Point{.x = baseX + (directionX * minProjection),
                                      .y = baseY + (directionY * minProjection)};
  const auto end = ad::types::Point{.x = baseX + (directionX * maxProjection),
                                    .y = baseY + (directionY * maxProjection)};

  return ObservedLine{.model = model,
                      .segment = ad::types::LineSegment{.start = start, .end = end},
                      .supportPointCount = supportPoints.size(),
                      .mse = mse};
}

auto extractOneLineCandidate(const std::vector<ScanPoint> &points,
                             const std::vector<LocalPointFeature> &features,
                             const std::vector<bool> &activeMask,
                             const std::vector<std::size_t> &activeIndices, std::mt19937 &rng,
                             const ad::localization::RansacLineAssociationModelConfig &config,
                             const int maxIterations) -> ExtractedLineCandidate {
  auto bestInliers = std::vector<std::size_t>{};
  bestInliers.reserve(activeIndices.size());

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    const auto sampled = sampleLocalPairIndices(rng, activeIndices, config.sampleNeighborWindow);
    if (!sampled) {
      continue;
    }

    const auto [firstIndex, secondIndex] = *sampled;
    const auto candidate =
        sampleLineFromPoints(points[firstIndex].point, points[secondIndex].point);
    if (!candidate) {
      continue;
    }

    auto inliers = collectHybridInlierIndices(points, features, activeMask, *candidate, config);
    if (inliers.size() > bestInliers.size()) {
      bestInliers = std::move(inliers);
    }
  }

  if (bestInliers.size() < config.minInlierPoints) {
    return ExtractedLineCandidate{.inlierIndices = std::move(bestInliers),
                                  .observedLine = std::nullopt};
  }

  auto supportPoints = std::vector<ad::types::Point>{};
  supportPoints.reserve(bestInliers.size());
  for (const auto index : bestInliers) {
    supportPoints.push_back(points[index].point);
  }

  const auto refined = fitLineFromPoints(supportPoints);
  if (!refined) {
    return ExtractedLineCandidate{.inlierIndices = std::move(bestInliers),
                                  .observedLine = std::nullopt};
  }

  return ExtractedLineCandidate{
      .inlierIndices = std::move(bestInliers),
      .observedLine = buildObservedLine(supportPoints, *refined, config.minExtractedSegmentLength)};
}

auto extractObservedLinesRansac(const ad::types::LidarScan &scan,
                                const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<ObservedLine> {
  const auto points = buildScanPoints(scan);
  const auto features = buildLocalPointFeatures(points, config);
  auto extracted = std::vector<ObservedLine>{};
  extracted.reserve(static_cast<std::size_t>(std::max(1, config.maxExtractedScanLines)));
  if (points.size() < 2U) {
    return extracted;
  }

  auto activeMask = std::vector<bool>(points.size(), true);
  auto activeIndices = std::vector<std::size_t>(points.size());
  std::iota(activeIndices.begin(), activeIndices.end(), 0U);
  auto remainingCount = points.size();

  auto rng = std::mt19937{static_cast<std::uint32_t>(points.size()) + kPointRansacSeedBias};
  const auto maxIterations = std::max(1, config.pointRansacMaxIterations);
  const auto maxScanLines = static_cast<std::size_t>(std::max(1, config.maxExtractedScanLines));
  const auto minRemaining = std::max<std::size_t>(config.minRemainingPoints, 2U);

  while (remainingCount >= minRemaining && extracted.size() < maxScanLines) {
    const auto candidate = extractOneLineCandidate(points, features, activeMask, activeIndices, rng,
                                                   config, maxIterations);
    if (candidate.inlierIndices.size() < config.minInlierPoints) {
      break;
    }

    std::size_t removedCount = 0U;
    for (const auto index : candidate.inlierIndices) {
      if (!activeMask[index]) {
        continue;
      }
      activeMask[index] = false;
      ++removedCount;
      --remainingCount;
    }
    activeIndices.erase(std::remove_if(activeIndices.begin(), activeIndices.end(),
                                       [&](const std::size_t index) { return !activeMask[index]; }),
                        activeIndices.end());

    if (removedCount == 0U) {
      break;
    }

    if (candidate.observedLine) {
      extracted.push_back(*candidate.observedLine);
    }
  }

  return extracted;
}

auto segmentOverlapsInMap(const ObservedLine &observed, const ad::types::Pose &pose,
                          const ad::localization::observation_model::util::MapLine &mapLine,
                          const double margin) -> bool {
  const auto startMap = transformPointToMap(observed.segment.start, pose);
  const auto endMap = transformPointToMap(observed.segment.end, pose);
  const auto startProjection =
      (mapLine.directionX * startMap.x) + (mapLine.directionY * startMap.y);
  const auto endProjection = (mapLine.directionX * endMap.x) + (mapLine.directionY * endMap.y);
  const auto observedMin = std::min(startProjection, endProjection);
  const auto observedMax = std::max(startProjection, endProjection);
  return observedMax >= (mapLine.minProjection - margin) &&
         observedMin <= (mapLine.maxProjection + margin);
}

auto computeMahalanobisDistanceSquared(const ad::types::Pose &observedPose,
                                       const ad::types::Pose &predictedPose,
                                       const ad::localization::CovarianceMatrix &covariance)
    -> std::optional<double> {
  const auto inverse = covariance.inverse();
  if (!inverse.allFinite()) {
    return std::nullopt;
  }

  Eigen::Vector3d delta;
  delta << observedPose.x - predictedPose.x, observedPose.y - predictedPose.y,
      ad::localization::util::normalizeAngle(observedPose.theta - predictedPose.theta);
  const auto distance = delta.transpose() * inverse * delta;
  if (!std::isfinite(distance)) {
    return std::nullopt;
  }
  return distance;
}

auto solvePoseFromLinePairs(const ObservedLine &observedFirst, const ObservedLine &observedSecond,
                            const ad::localization::observation_model::util::MapLine &mapFirst,
                            const ad::localization::observation_model::util::MapLine &mapSecond,
                            const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::optional<ad::types::Pose> {
  const auto thetaFirst =
      ad::localization::util::normalizeAngle(mapFirst.model.alpha - observedFirst.model.alpha);
  const auto thetaSecond =
      ad::localization::util::normalizeAngle(mapSecond.model.alpha - observedSecond.model.alpha);

  const auto thetaDiff = ad::localization::util::normalizeAngle(thetaSecond - thetaFirst);
  if (std::abs(thetaDiff) > config.lineAngleThreshold) {
    return std::nullopt;
  }

  const auto theta = ad::localization::util::normalizeAngle(thetaFirst + (0.5 * thetaDiff));

  const auto n1x = std::cos(mapFirst.model.alpha);
  const auto n1y = std::sin(mapFirst.model.alpha);
  const auto n2x = std::cos(mapSecond.model.alpha);
  const auto n2y = std::sin(mapSecond.model.alpha);
  const auto determinant = (n1x * n2y) - (n1y * n2x);
  if (std::abs(determinant) < std::max(kEpsilon, config.parallelRejectThreshold)) {
    return std::nullopt;
  }

  const auto rhsFirst = mapFirst.model.rho - observedFirst.model.rho;
  const auto rhsSecond = mapSecond.model.rho - observedSecond.model.rho;
  const auto x = ((rhsFirst * n2y) - (n1y * rhsSecond)) / determinant;
  const auto y = ((n1x * rhsSecond) - (rhsFirst * n2x)) / determinant;

  return ad::types::Pose{.x = x, .y = y, .theta = theta};
}

auto evaluatePoseMatches(
    const ad::types::Pose &pose, const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<std::pair<std::size_t, std::size_t>> {
  struct Candidate {
    std::size_t observedIndex;
    std::size_t mapIndex;
    double cost;
  };

  auto rawCandidates = std::vector<Candidate>{};
  rawCandidates.reserve(observedLines.size());

  for (std::size_t observedIndex = 0; observedIndex < observedLines.size(); ++observedIndex) {
    const auto transformed = transformLineToMap(observedLines[observedIndex].model, pose);

    auto bestMapIndex = mapLines.size();
    auto bestCost = std::numeric_limits<double>::infinity();
    for (std::size_t mapIndex = 0; mapIndex < mapLines.size(); ++mapIndex) {
      const auto &mapLine = mapLines[mapIndex];
      const auto angleDiff =
          std::abs(ad::localization::util::normalizeAngle(transformed.alpha - mapLine.model.alpha));
      if (angleDiff > config.lineAngleThreshold) {
        continue;
      }

      const auto rhoDiff = std::abs(transformed.rho - mapLine.model.rho);
      if (rhoDiff > config.lineRhoThreshold) {
        continue;
      }

      if (!segmentOverlapsInMap(observedLines[observedIndex], pose, mapLine,
                                config.segmentMargin)) {
        continue;
      }

      const auto cost = (angleDiff / std::max(kEpsilon, config.lineAngleThreshold)) +
                        (rhoDiff / std::max(kEpsilon, config.lineRhoThreshold));
      if (cost < bestCost) {
        bestCost = cost;
        bestMapIndex = mapIndex;
      }
    }

    if (bestMapIndex < mapLines.size()) {
      rawCandidates.push_back(
          Candidate{.observedIndex = observedIndex, .mapIndex = bestMapIndex, .cost = bestCost});
    }
  }

  std::sort(rawCandidates.begin(), rawCandidates.end(),
            [](const Candidate &left, const Candidate &right) { return left.cost < right.cost; });

  auto matches = std::vector<std::pair<std::size_t, std::size_t>>{};
  matches.reserve(rawCandidates.size());
  auto mapUsed = std::vector<bool>(mapLines.size(), false);
  for (const auto &candidate : rawCandidates) {
    if (mapUsed[candidate.mapIndex]) {
      continue;
    }
    mapUsed[candidate.mapIndex] = true;
    matches.emplace_back(candidate.observedIndex, candidate.mapIndex);
  }

  return matches;
}

auto runAssociationRansac(
    const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::types::Pose &predictedPose,
    const ad::localization::CovarianceMatrix &predictedCovariance,
    const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::optional<PoseMatchResult> {
  if (observedLines.size() < 2U || mapLines.size() < 2U) {
    return std::nullopt;
  }

  const auto baseSeed =
      buildAssociationRansacSeed(observedLines.size(), mapLines.size(), predictedPose);
  auto observedRng = std::mt19937{baseSeed ^ 0x9e3779b9U};
  auto mapRng = std::mt19937{baseSeed ^ 0x85ebca6bU};
  const auto iterations = std::max(1, config.translationRansacMaxIterations);

  auto best = std::optional<PoseMatchResult>{};
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const auto observedSample = sampleDistinctIndices(observedRng, observedLines.size());
    const auto mapSample = sampleDistinctIndices(mapRng, mapLines.size());
    if (!observedSample || !mapSample) {
      continue;
    }

    const auto [observedFirstIndex, observedSecondIndex] = *observedSample;
    const auto [mapFirstIndex, mapSecondIndex] = *mapSample;

    const auto pose = solvePoseFromLinePairs(
        observedLines[observedFirstIndex], observedLines[observedSecondIndex],
        mapLines[mapFirstIndex], mapLines[mapSecondIndex], config);
    if (!pose) {
      continue;
    }

    const auto maha = computeMahalanobisDistanceSquared(*pose, predictedPose, predictedCovariance);
    if (!maha) {
      continue;
    }
    if (*maha > config.contextGateThreshold) {
      continue;
    }

    auto pairs = evaluatePoseMatches(*pose, observedLines, mapLines, config);
    if (pairs.size() < config.minPoseInliers) {
      continue;
    }

    if (!best || pairs.size() > best->pairs.size() ||
        (pairs.size() == best->pairs.size() && *maha < best->mahalanobisDistanceSquared)) {
      best.emplace(PoseMatchResult{
          .pose = *pose, .pairs = std::move(pairs), .mahalanobisDistanceSquared = *maha});
    }
  }

  return best;
}

auto buildEkfUpdateFromPairs(
    const std::vector<std::pair<std::size_t, std::size_t>> &pairs,
    const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::types::Pose &predictedPose,
    const ad::localization::CovarianceMatrix &predictedCovariance,
    const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::optional<ad::localization::ObservationUpdateInput> {
  auto observations = std::vector<ad::localization::observation_model::util::LineObservation>{};
  observations.reserve(pairs.size());

  std::size_t gatePassed = 0U;
  for (const auto &[observedIndex, mapIndex] : pairs) {
    auto observation = ad::localization::observation_model::util::makeExpectedLine(
        mapLines[mapIndex].model, predictedPose);
    observation.observed = observedLines[observedIndex].model;

    ad::localization::observation_model::util::applyObservationNoiseFromMse(
        observation,
        ad::localization::observation_model::util::ObservationNoiseConfig{
            .measurementNoiseRange = config.measurementNoiseRange,
            .measurementNoiseAngle = config.measurementNoiseAngle},
        static_cast<double>(observedLines[observedIndex].supportPointCount),
        observedLines[observedIndex].mse);

    if (config.useEkfGate &&
        !ad::localization::observation_model::util::gateLineObservation(
            observation,
            ad::localization::observation_model::util::ObservationGateConfig{
                .covariance = predictedCovariance, .threshold = config.gateThreshold})) {
      continue;
    }

    ++gatePassed;
    observations.push_back(observation);
  }

  if (observations.size() < config.minObservations) {
    return std::nullopt;
  }

  const auto score =
      !pairs.empty() ? static_cast<double>(gatePassed) / static_cast<double>(pairs.size()) : 0.0;
  return ad::localization::observation_model::util::buildMeasurementData(observations, score);
}

} // namespace

namespace ad::localization {

RansacLineAssociationModel::RansacLineAssociationModel(
    std::vector<observation_model::util::MapLine> mapLines,
    observation_model::util::MapSignature signature, RansacLineAssociationModelConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)) {}

auto RansacLineAssociationModel::create(const types::MapData &map,
                                        RansacLineAssociationModelConfig config)
    -> Result<std::unique_ptr<RansacLineAssociationModel>> {
  const auto signature = observation_model::util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = line_extractor::extractMapLinesFromMap(map, config.mapLineExtraction);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto model =
      std::make_unique<RansacLineAssociationModel>(std::move(*mapLines), *signature, config);
  return {std::move(model)};
}

auto RansacLineAssociationModel::buildUpdateInput(const types::LidarScan &scan,
                                                  const types::MapData &map,
                                                  const types::Pose &predictedPose,
                                                  const CovarianceMatrix &predictedCovariance) const
    -> Result<std::optional<ObservationUpdateInput>> {
  if (!observation_model::util::signatureMatches(mapSignature_, map)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map does not match precomputed line features."});
  }

  if (scan.ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan has no ranges."});
  }

  const auto observedLines = extractObservedLinesRansac(scan, config_);
  if (observedLines.size() < config_.minObservations) {
    std::cerr << "[ransac_diag] stage=extract reject=min_observations extracted="
              << observedLines.size() << "\n";
    return {std::nullopt};
  }

  const auto bestMatch =
      runAssociationRansac(observedLines, mapLines_, predictedPose, predictedCovariance, config_);
  if (!bestMatch) {
    std::cerr << "[ransac_diag] stage=associate reject=no_consensus extracted="
              << observedLines.size() << "\n";
    return {std::nullopt};
  }

  const auto update = buildEkfUpdateFromPairs(bestMatch->pairs, observedLines, mapLines_,
                                              predictedPose, predictedCovariance, config_);
  if (!update) {
    std::cerr << "[ransac_diag] stage=ekf reject=insufficient_after_gate pairs="
              << bestMatch->pairs.size() << "\n";
    return {std::nullopt};
  }

  std::cerr << "[ransac_diag] accepted=1 extracted=" << observedLines.size()
            << " pairs=" << bestMatch->pairs.size()
            << " maha=" << bestMatch->mahalanobisDistanceSquared << "\n";
  return {update};
}

} // namespace ad::localization

#include "ransac_line_association_model.hpp"

#include "../localizer_util.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;
constexpr std::uint32_t kPointRansacSeedBias = 17U;
constexpr std::uint32_t kPoseRansacSeedMultiplier = 31U;
constexpr int kDistinctSampleRetryCount = 8;
constexpr double kClusterSplitDistanceMultiplier = 3.0;
constexpr double kMahaPenalty = 1e9;

struct ObservedLine {
  ad::localization::observation_model::util::LineModel model;
  ad::types::LineSegment segment;
  std::size_t supportPointCount;
  double mse;
};

struct PoseHypothesis {
  double x;
  double y;
  double theta;
  std::vector<std::pair<std::size_t, std::size_t>> matches;
  double coarseScore;
  double refinementCost;
  double mahalanobisDistanceSquared;
};

struct TranslationConstraint {
  std::size_t observedIndex;
  std::size_t mapIndex;
  double rhs;
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

auto serializeObservedSegments(const std::vector<ObservedLine> &observedLines) -> std::string {
  auto stream = std::ostringstream{};
  for (std::size_t index = 0; index < observedLines.size(); ++index) {
    const auto &segment = observedLines[index].segment;
    stream << segment.start.x << ':' << segment.start.y << ':' << segment.end.x << ':'
           << segment.end.y;
    if ((index + 1U) < observedLines.size()) {
      stream << '|';
    }
  }
  return stream.str();
}

auto evaluatePoseHypothesis(
    const ad::types::Pose &pose, const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<std::pair<std::size_t, std::size_t>> {
  auto matches = std::vector<std::pair<std::size_t, std::size_t>>{};
  matches.reserve(observedLines.size());

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
      matches.emplace_back(observedIndex, bestMapIndex);
    }
  }

  return matches;
}

auto toPose(const PoseHypothesis &hypothesis) -> ad::types::Pose {
  return ad::types::Pose{.x = hypothesis.x, .y = hypothesis.y, .theta = hypothesis.theta};
}

auto buildOrientationCandidates(
    const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config) -> std::vector<double> {
  const auto binSize = std::max(kEpsilon, config.orientationBinSize);
  const auto binCount =
      static_cast<std::size_t>(std::max(8.0, std::ceil((2.0 * std::numbers::pi) / binSize)));
  auto bins = std::vector<std::size_t>(binCount, 0U);

  for (const auto &observed : observedLines) {
    for (const auto &mapLine : mapLines) {
      const auto theta =
          ad::localization::util::normalizeAngle(mapLine.model.alpha - observed.model.alpha);
      const auto shifted = theta + std::numbers::pi;
      const auto rawIndex = static_cast<std::size_t>(
          std::clamp(std::floor(shifted / binSize), 0.0, static_cast<double>(binCount - 1U)));
      ++bins[rawIndex];
    }
  }

  struct Peak {
    std::size_t index;
    std::size_t votes;
  };
  auto peaks = std::vector<Peak>{};
  peaks.reserve(binCount);
  for (std::size_t index = 0; index < binCount; ++index) {
    const auto prev = index == 0U ? binCount - 1U : index - 1U;
    const auto next = (index + 1U) % binCount;
    if (bins[index] < config.orientationPeakMinVotes) {
      continue;
    }
    if (bins[index] >= bins[prev] && bins[index] >= bins[next]) {
      peaks.push_back(Peak{.index = index, .votes = bins[index]});
    }
  }
  std::sort(peaks.begin(), peaks.end(),
            [](const Peak &left, const Peak &right) { return left.votes > right.votes; });

  auto orientations = std::vector<double>{};
  orientations.reserve(std::min(config.maxOrientationCandidates, peaks.size()));
  for (std::size_t idx = 0;
       idx < peaks.size() && orientations.size() < config.maxOrientationCandidates; ++idx) {
    const auto theta = ad::localization::util::normalizeAngle(
        (-std::numbers::pi) + (binSize * (static_cast<double>(peaks[idx].index) + 0.5)));
    orientations.push_back(theta);
  }

  if (orientations.empty()) {
    orientations.push_back(0.0);
  }
  return orientations;
}

auto buildTranslationConstraintsForTheta(
    const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config, const double theta)
    -> std::vector<TranslationConstraint> {
  auto constraints = std::vector<TranslationConstraint>{};
  constraints.reserve(observedLines.size() * mapLines.size());

  for (std::size_t observedIndex = 0; observedIndex < observedLines.size(); ++observedIndex) {
    const auto transformedAlpha =
        ad::localization::util::normalizeAngle(observedLines[observedIndex].model.alpha + theta);
    for (std::size_t mapIndex = 0; mapIndex < mapLines.size(); ++mapIndex) {
      const auto angleDiff = std::abs(ad::localization::util::normalizeAngle(
          transformedAlpha - mapLines[mapIndex].model.alpha));
      if (angleDiff > config.lineAngleThreshold) {
        continue;
      }

      constraints.push_back(TranslationConstraint{.observedIndex = observedIndex,
                                                  .mapIndex = mapIndex,
                                                  .rhs = mapLines[mapIndex].model.rho -
                                                         observedLines[observedIndex].model.rho});
    }
  }

  return constraints;
}

auto solveTranslationFromConstraints(
    const TranslationConstraint &firstConstraint, const TranslationConstraint &secondConstraint,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config, const double theta)
    -> std::optional<ad::types::Pose> {
  const auto &firstMap = mapLines[firstConstraint.mapIndex];
  const auto &secondMap = mapLines[secondConstraint.mapIndex];

  const auto n1x = std::cos(firstMap.model.alpha);
  const auto n1y = std::sin(firstMap.model.alpha);
  const auto n2x = std::cos(secondMap.model.alpha);
  const auto n2y = std::sin(secondMap.model.alpha);
  const auto determinant = (n1x * n2y) - (n1y * n2x);
  if (std::abs(determinant) < std::max(kEpsilon, config.parallelRejectThreshold)) {
    return std::nullopt;
  }

  const auto x = ((firstConstraint.rhs * n2y) - (n1y * secondConstraint.rhs)) / determinant;
  const auto y = ((n1x * secondConstraint.rhs) - (firstConstraint.rhs * n2x)) / determinant;
  return ad::types::Pose{.x = x, .y = y, .theta = ad::localization::util::normalizeAngle(theta)};
}

auto runTranslationRansacForTheta(
    const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config, const double theta,
    std::mt19937 &rng) -> std::vector<PoseHypothesis> {
  const auto constraints =
      buildTranslationConstraintsForTheta(observedLines, mapLines, config, theta);
  auto candidates = std::vector<PoseHypothesis>{};
  if (constraints.size() < 2U) {
    return candidates;
  }

  const auto maxIterations = std::max(1, config.translationRansacMaxIterations);
  candidates.reserve(static_cast<std::size_t>(maxIterations));

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    const auto sampled = sampleDistinctIndices(rng, constraints.size());
    if (!sampled) {
      continue;
    }
    const auto [firstIndex, secondIndex] = *sampled;
    const auto &firstConstraint = constraints[firstIndex];
    const auto &secondConstraint = constraints[secondIndex];
    if (firstConstraint.mapIndex == secondConstraint.mapIndex) {
      continue;
    }

    const auto pose =
        solveTranslationFromConstraints(firstConstraint, secondConstraint, mapLines, config, theta);
    if (!pose) {
      continue;
    }

    auto matches = evaluatePoseHypothesis(*pose, observedLines, mapLines, config);
    if (matches.size() < config.minPoseInliers) {
      continue;
    }

    const auto coarseScore = static_cast<double>(matches.size());
    candidates.push_back(PoseHypothesis{.x = pose->x,
                                        .y = pose->y,
                                        .theta = pose->theta,
                                        .matches = std::move(matches),
                                        .coarseScore = coarseScore,
                                        .refinementCost = std::numeric_limits<double>::infinity(),
                                        .mahalanobisDistanceSquared = kMahaPenalty});
  }

  return candidates;
}

auto clusterHypotheses(const std::vector<PoseHypothesis> &raw,
                       const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<PoseHypothesis> {
  if (raw.empty()) {
    return {};
  }

  auto sorted = raw;
  std::sort(sorted.begin(), sorted.end(),
            [](const PoseHypothesis &left, const PoseHypothesis &right) {
              if (left.matches.size() != right.matches.size()) {
                return left.matches.size() > right.matches.size();
              }
              return left.coarseScore > right.coarseScore;
            });

  auto clustered = std::vector<PoseHypothesis>{};
  clustered.reserve(std::min(sorted.size(), config.maxCoarseHypotheses));

  for (const auto &candidate : sorted) {
    bool isClose = false;
    for (const auto &selected : clustered) {
      const auto positionDistance = std::hypot(candidate.x - selected.x, candidate.y - selected.y);
      const auto angleDistance =
          std::abs(ad::localization::util::normalizeAngle(candidate.theta - selected.theta));
      if (positionDistance <= config.clusterPositionThreshold &&
          angleDistance <= config.clusterAngleThreshold) {
        isClose = true;
        break;
      }
    }

    if (!isClose) {
      clustered.push_back(candidate);
      if (clustered.size() >= config.maxCoarseHypotheses) {
        break;
      }
    }
  }

  return clustered;
}

auto coarseSearchHypotheses(
    const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<PoseHypothesis> {
  if (observedLines.size() < 2U || mapLines.size() < 2U) {
    return {};
  }

  auto rng = std::mt19937{static_cast<std::uint32_t>(
      (observedLines.size() * kPoseRansacSeedMultiplier) + mapLines.size())};
  const auto orientations = buildOrientationCandidates(observedLines, mapLines, config);
  auto allCandidates = std::vector<PoseHypothesis>{};

  for (const auto theta : orientations) {
    auto candidates = runTranslationRansacForTheta(observedLines, mapLines, config, theta, rng);
    allCandidates.insert(allCandidates.end(), std::make_move_iterator(candidates.begin()),
                         std::make_move_iterator(candidates.end()));
  }

  return clusterHypotheses(allCandidates, config);
}

auto refineHypothesisGaussNewton(
    const PoseHypothesis &inputHypothesis, const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config) -> PoseHypothesis {
  auto poseX = inputHypothesis.x;
  auto poseY = inputHypothesis.y;
  auto poseTheta = inputHypothesis.theta;
  const auto maxIterations = std::max(1, config.refinementMaxIterations);

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    Eigen::Matrix3d jtj = Eigen::Matrix3d::Zero();
    Eigen::Vector3d jtr = Eigen::Vector3d::Zero();

    for (const auto &[observedIndex, mapIndex] : inputHypothesis.matches) {
      const auto &observed = observedLines[observedIndex];
      const auto &mapLine = mapLines[mapIndex];
      const auto nx = std::cos(mapLine.model.alpha);
      const auto ny = std::sin(mapLine.model.alpha);
      const auto weight =
          std::max(kEpsilon, std::hypot(observed.segment.end.x - observed.segment.start.x,
                                        observed.segment.end.y - observed.segment.start.y));
      const auto cosTheta = std::cos(poseTheta);
      const auto sinTheta = std::sin(poseTheta);

      const auto accumulate = [&](const ad::types::Point &localPoint) {
        const auto transformedX = poseX + (cosTheta * localPoint.x) - (sinTheta * localPoint.y);
        const auto transformedY = poseY + (sinTheta * localPoint.x) + (cosTheta * localPoint.y);
        const auto residual = (nx * transformedX) + (ny * transformedY) - mapLine.model.rho;

        const auto dxdTheta = (-sinTheta * localPoint.x) - (cosTheta * localPoint.y);
        const auto dydTheta = (cosTheta * localPoint.x) - (sinTheta * localPoint.y);
        Eigen::Vector3d jacobian;
        jacobian << nx, ny, (nx * dxdTheta) + (ny * dydTheta);

        jtj += weight * (jacobian * jacobian.transpose());
        jtr += weight * (jacobian * residual);
      };

      accumulate(observed.segment.start);
      accumulate(observed.segment.end);
    }

    jtj += config.refinementDamping * Eigen::Matrix3d::Identity();
    const auto delta = jtj.ldlt().solve(-jtr);
    if (!delta.allFinite()) {
      break;
    }

    poseX += delta(0);
    poseY += delta(1);
    poseTheta = ad::localization::util::normalizeAngle(poseTheta + delta(2));
    if (delta.norm() < config.refinementStepTolerance) {
      break;
    }
  }

  const auto refinedPose = ad::types::Pose{.x = poseX, .y = poseY, .theta = poseTheta};
  auto refinedMatches = evaluatePoseHypothesis(refinedPose, observedLines, mapLines, config);
  auto refinementCost = 0.0;
  for (const auto &[observedIndex, mapIndex] : refinedMatches) {
    const auto &observed = observedLines[observedIndex];
    const auto &mapLine = mapLines[mapIndex];
    const auto nx = std::cos(mapLine.model.alpha);
    const auto ny = std::sin(mapLine.model.alpha);
    const auto weight =
        std::max(kEpsilon, std::hypot(observed.segment.end.x - observed.segment.start.x,
                                      observed.segment.end.y - observed.segment.start.y));
    const auto start = transformPointToMap(observed.segment.start, refinedPose);
    const auto end = transformPointToMap(observed.segment.end, refinedPose);
    const auto eStart = (nx * start.x) + (ny * start.y) - mapLine.model.rho;
    const auto eEnd = (nx * end.x) + (ny * end.y) - mapLine.model.rho;
    refinementCost += weight * ((eStart * eStart) + (eEnd * eEnd));
  }

  return PoseHypothesis{.x = poseX,
                        .y = poseY,
                        .theta = poseTheta,
                        .matches = std::move(refinedMatches),
                        .coarseScore = inputHypothesis.coarseScore,
                        .refinementCost = refinementCost,
                        .mahalanobisDistanceSquared = kMahaPenalty};
}

auto refineHypotheses(
    const std::vector<PoseHypothesis> &coarseHypotheses,
    const std::vector<ObservedLine> &observedLines,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<PoseHypothesis> {
  auto refined = std::vector<PoseHypothesis>{};
  refined.reserve(coarseHypotheses.size());
  for (const auto &hypothesis : coarseHypotheses) {
    auto refinedHypothesis =
        refineHypothesisGaussNewton(hypothesis, observedLines, mapLines, config);
    if (refinedHypothesis.matches.size() >= config.minPoseInliers) {
      refined.push_back(std::move(refinedHypothesis));
    }
  }
  return refined;
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

auto selectFinalHypothesis(const std::vector<PoseHypothesis> &refinedHypotheses,
                           const ad::types::Pose &predictedPose,
                           const ad::localization::CovarianceMatrix &predictedCovariance,
                           const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::optional<PoseHypothesis> {
  auto best = std::optional<PoseHypothesis>{};

  for (const auto &hypothesis : refinedHypotheses) {
    const auto maha =
        computeMahalanobisDistanceSquared(toPose(hypothesis), predictedPose, predictedCovariance);
    if (!maha) {
      continue;
    }

    if (config.useContextGate && *maha > config.contextGateThreshold) {
      continue;
    }

    auto updated = hypothesis;
    updated.mahalanobisDistanceSquared = *maha;
    if (!best || updated.mahalanobisDistanceSquared < best->mahalanobisDistanceSquared) {
      best = std::move(updated);
    }
  }

  return best;
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
  const auto stage1ExtractedCount = observedLines.size();
  const auto stage1Segments = serializeObservedSegments(observedLines);
  if (observedLines.size() < config_.minObservations) {
    std::cerr << "[ransac_diag] stage1_extracted=" << stage1ExtractedCount
              << " stage2_matches=0 gate_passed=0 accepted=0 reject=stage1_min_observations"
              << " stage1_segments=" << stage1Segments << "\n";
    return {std::nullopt};
  }

  const auto coarseHypotheses = coarseSearchHypotheses(observedLines, mapLines_, config_);
  if (coarseHypotheses.empty()) {
    std::cerr << "[ransac_diag] stage1_extracted=" << stage1ExtractedCount
              << " phase2_candidates=0 phase3_refined=0 phase4_gate_passed=0 accepted=0"
              << " reject=phase2_no_hypothesis"
              << " stage1_segments=" << stage1Segments << "\n";
    return {std::nullopt};
  }

  const auto refinedHypotheses =
      refineHypotheses(coarseHypotheses, observedLines, mapLines_, config_);
  if (refinedHypotheses.empty()) {
    std::cerr << "[ransac_diag] stage1_extracted=" << stage1ExtractedCount
              << " phase2_candidates=" << coarseHypotheses.size()
              << " phase3_refined=0 phase4_gate_passed=0 accepted=0"
              << " reject=phase3_no_refined"
              << " stage1_segments=" << stage1Segments << "\n";
    return {std::nullopt};
  }

  std::size_t phase4GatePassed = 0U;
  for (const auto &candidate : refinedHypotheses) {
    const auto maha =
        computeMahalanobisDistanceSquared(toPose(candidate), predictedPose, predictedCovariance);
    if (!maha) {
      continue;
    }
    if (!config_.useContextGate || *maha <= config_.contextGateThreshold) {
      ++phase4GatePassed;
    }
  }

  const auto bestHypothesis =
      selectFinalHypothesis(refinedHypotheses, predictedPose, predictedCovariance, config_);
  if (!bestHypothesis || bestHypothesis->matches.size() < config_.minObservations) {
    std::cerr << "[ransac_diag] stage1_extracted=" << stage1ExtractedCount
              << " phase2_candidates=" << coarseHypotheses.size()
              << " phase3_refined=" << refinedHypotheses.size()
              << " phase4_gate_passed=" << phase4GatePassed << " accepted=0"
              << " reject=phase4_selection"
              << " stage1_segments=" << stage1Segments << "\n";
    return {std::nullopt};
  }

  auto observations = std::vector<observation_model::util::LineObservation>{};
  observations.reserve(bestHypothesis->matches.size());
  std::size_t gatePassed = 0;

  for (const auto &[observedIndex, mapIndex] : bestHypothesis->matches) {
    auto observation =
        observation_model::util::makeExpectedLine(mapLines_[mapIndex].model, predictedPose);
    observation.observed = observedLines[observedIndex].model;

    observation_model::util::applyObservationNoiseFromMse(
        observation,
        observation_model::util::ObservationNoiseConfig{
            .measurementNoiseRange = config_.measurementNoiseRange,
            .measurementNoiseAngle = config_.measurementNoiseAngle},
        static_cast<double>(observedLines[observedIndex].supportPointCount),
        observedLines[observedIndex].mse);

    if (config_.useEkfGate && !observation_model::util::gateLineObservation(
                                  observation, observation_model::util::ObservationGateConfig{
                                                   .covariance = predictedCovariance,
                                                   .threshold = config_.gateThreshold})) {
      continue;
    }

    ++gatePassed;
    observations.push_back(observation);
  }

  if (observations.size() < config_.minObservations) {
    std::cerr << "[ransac_diag] stage1_extracted=" << stage1ExtractedCount
              << " phase2_candidates=" << coarseHypotheses.size()
              << " phase3_refined=" << refinedHypotheses.size()
              << " phase4_gate_passed=" << phase4GatePassed << " gate_passed=" << gatePassed
              << " accepted=0 reject=gate_min_observations"
              << " stage1_segments=" << stage1Segments << "\n";
    return {std::nullopt};
  }

  const auto score =
      !bestHypothesis->matches.empty()
          ? static_cast<double>(gatePassed) / static_cast<double>(bestHypothesis->matches.size())
          : 0.0;
  std::cerr << "[ransac_diag] stage1_extracted=" << stage1ExtractedCount
            << " phase2_candidates=" << coarseHypotheses.size()
            << " phase3_refined=" << refinedHypotheses.size()
            << " phase4_gate_passed=" << phase4GatePassed << " gate_passed=" << gatePassed
            << " accepted=1 score=" << score
            << " best_maha=" << bestHypothesis->mahalanobisDistanceSquared
            << " stage1_segments=" << stage1Segments << "\n";
  return {observation_model::util::buildMeasurementData(observations, score)};
}

} // namespace ad::localization

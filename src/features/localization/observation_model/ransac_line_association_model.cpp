#include "ransac_line_association_model.hpp"

#include "../localizer_util.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <utility>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;
constexpr std::uint32_t kPointRansacSeedBias = 17U;
constexpr std::uint32_t kPoseRansacSeedMultiplier = 31U;
constexpr int kDistinctSampleRetryCount = 8;

struct ObservedLine {
  ad::localization::observation_model::util::LineModel model;
  ad::types::LineSegment segment;
  std::size_t supportPointCount;
  double mse;
};

struct PoseHypothesis {
  ad::types::Pose pose;
  std::vector<std::pair<std::size_t, std::size_t>> matches;
};

struct ExtractedLineCandidate {
  std::vector<std::size_t> inlierIndices;
  std::optional<ObservedLine> observedLine;
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

auto buildScanPoints(const ad::types::LidarScan &scan) -> std::vector<ad::types::Point> {
  auto points = std::vector<ad::types::Point>{};
  points.reserve(scan.ranges.size());

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }

    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    points.push_back(ad::types::Point{.x = range * std::cos(angle), .y = range * std::sin(angle)});
  }

  return points;
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

auto collectInlierIndices(const std::vector<ad::types::Point> &points,
                          const ad::localization::observation_model::util::LineModel &model,
                          const double distanceThreshold) -> std::vector<std::size_t> {
  auto inliers = std::vector<std::size_t>{};
  inliers.reserve(points.size());
  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);

  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto &point = points[index];
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - model.rho);
    if (distance <= distanceThreshold) {
      inliers.push_back(index);
    }
  }

  return inliers;
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

auto filterRemainingPoints(const std::vector<ad::types::Point> &remaining,
                           const std::vector<std::size_t> &inlierIndices)
    -> std::vector<ad::types::Point> {
  auto inlierFlags = std::vector<bool>(remaining.size(), false);
  for (const auto index : inlierIndices) {
    inlierFlags[index] = true;
  }

  auto filtered = std::vector<ad::types::Point>{};
  filtered.reserve(remaining.size() - inlierIndices.size());
  for (std::size_t index = 0; index < remaining.size(); ++index) {
    if (!inlierFlags[index]) {
      filtered.push_back(remaining[index]);
    }
  }
  return filtered;
}

auto extractOneLineCandidate(const std::vector<ad::types::Point> &remaining, std::mt19937 &rng,
                             const ad::localization::RansacLineAssociationModelConfig &config,
                             const int maxIterations) -> ExtractedLineCandidate {
  auto bestInliers = std::vector<std::size_t>{};
  bestInliers.reserve(remaining.size());

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    const auto sampled = sampleDistinctIndices(rng, remaining.size());
    if (!sampled) {
      continue;
    }

    const auto [firstIndex, secondIndex] = *sampled;
    const auto candidate = sampleLineFromPoints(remaining[firstIndex], remaining[secondIndex]);
    if (!candidate) {
      continue;
    }

    auto inliers = collectInlierIndices(remaining, *candidate, config.pointDistanceThreshold);
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
    supportPoints.push_back(remaining[index]);
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
  auto remaining = buildScanPoints(scan);
  auto extracted = std::vector<ObservedLine>{};
  extracted.reserve(static_cast<std::size_t>(std::max(1, config.maxExtractedScanLines)));

  auto rng = std::mt19937{static_cast<std::uint32_t>(remaining.size()) + kPointRansacSeedBias};
  const auto maxIterations = std::max(1, config.pointRansacMaxIterations);
  const auto maxScanLines = static_cast<std::size_t>(std::max(1, config.maxExtractedScanLines));
  const auto minRemaining = std::max<std::size_t>(config.minRemainingPoints, 2U);

  while (remaining.size() >= minRemaining && extracted.size() < maxScanLines) {
    const auto candidate = extractOneLineCandidate(remaining, rng, config, maxIterations);
    if (candidate.inlierIndices.size() < config.minInlierPoints) {
      break;
    }

    remaining = filterRemainingPoints(remaining, candidate.inlierIndices);
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

auto estimatePoseFromPairs(const ObservedLine &obsFirst,
                           const ad::localization::observation_model::util::MapLine &mapFirst,
                           const ObservedLine &obsSecond,
                           const ad::localization::observation_model::util::MapLine &mapSecond,
                           const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::optional<ad::types::Pose> {
  const auto thetaFirst =
      ad::localization::util::normalizeAngle(mapFirst.model.alpha - obsFirst.model.alpha);
  const auto thetaSecond =
      ad::localization::util::normalizeAngle(mapSecond.model.alpha - obsSecond.model.alpha);
  const auto thetaDiff = std::abs(ad::localization::util::normalizeAngle(thetaFirst - thetaSecond));
  if (thetaDiff > config.lineAngleThreshold) {
    return std::nullopt;
  }

  const auto theta = std::atan2(std::sin(thetaFirst) + std::sin(thetaSecond),
                                std::cos(thetaFirst) + std::cos(thetaSecond));

  const auto n1x = std::cos(mapFirst.model.alpha);
  const auto n1y = std::sin(mapFirst.model.alpha);
  const auto n2x = std::cos(mapSecond.model.alpha);
  const auto n2y = std::sin(mapSecond.model.alpha);
  const auto determinant = (n1x * n2y) - (n1y * n2x);
  if (std::abs(determinant) < std::max(kEpsilon, config.parallelRejectThreshold)) {
    return std::nullopt;
  }

  const auto rhsFirst = mapFirst.model.rho - obsFirst.model.rho;
  const auto rhsSecond = mapSecond.model.rho - obsSecond.model.rho;
  const auto poseX = ((rhsFirst * n2y) - (n1y * rhsSecond)) / determinant;
  const auto poseY = ((n1x * rhsSecond) - (rhsFirst * n2x)) / determinant;

  return ad::types::Pose{
      .x = poseX, .y = poseY, .theta = ad::localization::util::normalizeAngle(theta)};
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

auto runPoseRansac(const std::vector<ObservedLine> &observedLines,
                   const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
                   const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::optional<PoseHypothesis> {
  if (observedLines.size() < 2U || mapLines.size() < 2U) {
    return std::nullopt;
  }

  auto rng = std::mt19937{static_cast<std::uint32_t>(
      (observedLines.size() * kPoseRansacSeedMultiplier) + mapLines.size())};

  auto bestHypothesis = std::optional<PoseHypothesis>{};
  const auto maxIterations = std::max(1, config.poseRansacMaxIterations);

  const auto tryAssignment =
      [&](const std::size_t observedFirstIndex, const std::size_t observedSecondIndex,
          const std::size_t mapFirstIndex,
          const std::size_t mapSecondIndex) -> std::optional<PoseHypothesis> {
    const auto estimatedPose =
        estimatePoseFromPairs(observedLines[observedFirstIndex], mapLines[mapFirstIndex],
                              observedLines[observedSecondIndex], mapLines[mapSecondIndex], config);
    if (!estimatedPose) {
      return std::nullopt;
    }

    auto matches = evaluatePoseHypothesis(*estimatedPose, observedLines, mapLines, config);
    if (matches.size() < config.minPoseInliers) {
      return std::nullopt;
    }
    return PoseHypothesis{.pose = *estimatedPose, .matches = std::move(matches)};
  };

  const auto chooseBetter =
      [&](const std::optional<PoseHypothesis> &left,
          const std::optional<PoseHypothesis> &right) -> std::optional<PoseHypothesis> {
    if (left && right) {
      return left->matches.size() >= right->matches.size() ? left : right;
    }
    return left ? left : right;
  };

  for (int iteration = 0; iteration < maxIterations; ++iteration) {
    const auto observedPair = sampleDistinctIndices(rng, observedLines.size());
    if (!observedPair) {
      continue;
    }
    const auto mapPair = sampleDistinctIndices(rng, mapLines.size());
    if (!mapPair) {
      continue;
    }

    const auto [obsFirstIndex, obsSecondIndex] = *observedPair;
    const auto [mapFirstIndex, mapSecondIndex] = *mapPair;

    const auto direct = tryAssignment(obsFirstIndex, obsSecondIndex, mapFirstIndex, mapSecondIndex);
    const auto reverseAssignment =
        tryAssignment(obsFirstIndex, obsSecondIndex, mapSecondIndex, mapFirstIndex);
    const auto candidate = chooseBetter(direct, reverseAssignment);
    if (!candidate) {
      continue;
    }

    if (!bestHypothesis || candidate->matches.size() > bestHypothesis->matches.size()) {
      bestHypothesis.emplace(*candidate);
    }
  }

  return bestHypothesis;
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
    return {std::nullopt};
  }

  const auto bestHypothesis = runPoseRansac(observedLines, mapLines_, config_);
  if (!bestHypothesis || bestHypothesis->matches.size() < config_.minObservations) {
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
    return {std::nullopt};
  }

  const auto score =
      !bestHypothesis->matches.empty()
          ? static_cast<double>(gatePassed) / static_cast<double>(bestHypothesis->matches.size())
          : 0.0;
  return {observation_model::util::buildMeasurementData(observations, score)};
}

} // namespace ad::localization

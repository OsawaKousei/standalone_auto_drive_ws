#include "hough_ransac_observation_model.hpp"

#include "hough_line_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <vector>

namespace {

constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;
constexpr double kDefaultCandidateAngleGate = 0.20;
constexpr double kDefaultCandidateRhoGate = 0.50;
constexpr double kDefaultInlierAngleThreshold = 0.22;
constexpr double kMinSampleAngleSeparation = 0.10;
constexpr double kMinOrientationDiversity = 0.01;
constexpr std::size_t kMaxCandidatesPerScanLine = 4U;
constexpr std::size_t kMaxTotalCandidates = 160U;
constexpr double kSolveEpsilon = 1e-8;

using Mat3 = ad::localization::CovarianceMatrix;

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

struct CandidatePair {
  std::size_t scanLineIndex;
  std::size_t mapLineIndex;
  double coarseScore;
};

struct MatchedPair {
  std::size_t scanLineIndex;
  std::size_t mapLineIndex;
  double angleResidual;
  double rhoResidual;
  double score;
};

auto fnv1aMix(std::uint64_t hash, std::uint64_t value) -> std::uint64_t {
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  hash ^= value;
  hash *= kPrime;
  return hash;
}

auto quantizeToMilli(double value) -> std::int64_t {
  return static_cast<std::int64_t>(std::llround(value * 1000.0));
}

auto buildRansacSeed(const ad::types::Pose &pose, std::size_t scanLineCount,
                     std::size_t candidateCount) -> std::uint32_t {
  std::uint64_t hash = 1469598103934665603ULL;
  hash = fnv1aMix(hash, static_cast<std::uint64_t>(scanLineCount));
  hash = fnv1aMix(hash, static_cast<std::uint64_t>(candidateCount));
  hash = fnv1aMix(hash, static_cast<std::uint64_t>(quantizeToMilli(pose.x)));
  hash = fnv1aMix(hash, static_cast<std::uint64_t>(quantizeToMilli(pose.y)));
  hash = fnv1aMix(hash, static_cast<std::uint64_t>(quantizeToMilli(pose.theta)));
  return static_cast<std::uint32_t>((hash >> 32U) ^ (hash & 0xffffffffULL));
}

auto collectScanPoints(const ad::types::LidarScan &scan) -> std::vector<ad::types::Point> {
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

auto segmentLength(const ad::localization::util::MapLine &line) -> double {
  const auto deltaX = line.segment.end.x - line.segment.start.x;
  const auto deltaY = line.segment.end.y - line.segment.start.y;
  return std::hypot(deltaX, deltaY);
}

auto transformLocalLineToMap(const ad::localization::util::LineModel &localLine,
                             const ad::types::Pose &pose) -> ad::localization::util::LineModel {
  const auto alphaMap = ad::localization::util::normalizeAngle(localLine.alpha + pose.theta);
  const auto rhoMap = localLine.rho + (pose.x * std::cos(alphaMap)) + (pose.y * std::sin(alphaMap));
  return ad::localization::util::toLineModel(
      ad::localization::util::LineModel{.rho = rhoMap, .alpha = alphaMap});
}

auto candidateGates(const ad::localization::HoughObservationModelConfig &config)
    -> std::pair<double, double> {
  const auto angleGate = std::max(kDefaultCandidateAngleGate, config.hough.mergeTheta * 2.5);
  const auto rhoGate = std::max(kDefaultCandidateRhoGate, config.maxAssociationDistance * 2.0);
  return {angleGate, rhoGate};
}

auto buildCandidatePairs(const std::vector<ad::localization::util::MapLine> &scanLines,
                         const std::vector<ad::localization::util::MapLine> &mapLines,
                         const ad::types::Pose &predictedPose,
                         const ad::localization::HoughRansacObservationModelConfig &config)
    -> std::vector<CandidatePair> {
  auto candidates = std::vector<CandidatePair>{};
  const auto [angleGate, rhoGate] = candidateGates(config.houghObservation);

  for (std::size_t scanIndex = 0; scanIndex < scanLines.size(); ++scanIndex) {
    const auto &scanLine = scanLines[scanIndex];
    if (segmentLength(scanLine) < config.ransac.minInlierSpan) {
      continue;
    }

    const auto predictedMapLine = transformLocalLineToMap(scanLine.model, predictedPose);

    auto perScan = std::vector<CandidatePair>{};
    perScan.reserve(mapLines.size());

    for (std::size_t mapIndex = 0; mapIndex < mapLines.size(); ++mapIndex) {
      const auto &mapLine = mapLines[mapIndex];
      const auto angleResidual = std::abs(
          ad::localization::util::normalizeAngle(predictedMapLine.alpha - mapLine.model.alpha));
      if (angleResidual > angleGate) {
        continue;
      }

      const auto rhoResidual = std::abs(predictedMapLine.rho - mapLine.model.rho);
      if (rhoResidual > rhoGate) {
        continue;
      }

      const auto coarseScore = angleResidual + (rhoResidual / rhoGate);
      perScan.push_back(CandidatePair{
          .scanLineIndex = scanIndex, .mapLineIndex = mapIndex, .coarseScore = coarseScore});
    }

    std::sort(perScan.begin(), perScan.end(), [](const auto &left, const auto &right) -> bool {
      return left.coarseScore < right.coarseScore;
    });

    const auto limit = std::min(kMaxCandidatesPerScanLine, perScan.size());
    for (std::size_t index = 0; index < limit; ++index) {
      candidates.push_back(perScan[index]);
      if (candidates.size() >= kMaxTotalCandidates) {
        return candidates;
      }
    }
  }

  return candidates;
}

auto drawSamplePair(std::mt19937 &generator, std::size_t candidateCount)
    -> std::optional<std::pair<std::size_t, std::size_t>> {
  if (candidateCount < 2U) {
    return std::nullopt;
  }

  std::uniform_int_distribution<std::size_t> distribution(0U, candidateCount - 1U);
  auto first = distribution(generator);
  auto second = distribution(generator);
  for (int retry = 0; retry < 8 && second == first; ++retry) {
    second = distribution(generator);
  }
  if (first == second) {
    return std::nullopt;
  }
  return std::pair<std::size_t, std::size_t>{first, second};
}

auto estimatePoseFromTwoPairs(const CandidatePair &leftPair, const CandidatePair &rightPair,
                              const std::vector<ad::localization::util::MapLine> &scanLines,
                              const std::vector<ad::localization::util::MapLine> &mapLines)
    -> std::optional<ad::types::Pose> {
  if (leftPair.scanLineIndex == rightPair.scanLineIndex ||
      leftPair.mapLineIndex == rightPair.mapLineIndex) {
    return std::nullopt;
  }

  const auto &scanLeft = scanLines[leftPair.scanLineIndex].model;
  const auto &scanRight = scanLines[rightPair.scanLineIndex].model;
  const auto &mapLeft = mapLines[leftPair.mapLineIndex].model;
  const auto &mapRight = mapLines[rightPair.mapLineIndex].model;

  const auto thetaLeft = ad::localization::util::normalizeAngle(mapLeft.alpha - scanLeft.alpha);
  const auto thetaRight = ad::localization::util::normalizeAngle(mapRight.alpha - scanRight.alpha);
  const auto thetaGap = std::abs(ad::localization::util::normalizeAngle(thetaLeft - thetaRight));
  if (thetaGap > 0.35) {
    return std::nullopt;
  }

  const auto sampleMapAngleGap =
      std::abs(ad::localization::util::normalizeAngle(mapLeft.alpha - mapRight.alpha));
  if (sampleMapAngleGap < kMinSampleAngleSeparation) {
    return std::nullopt;
  }

  const auto sinSum = std::sin(thetaLeft) + std::sin(thetaRight);
  const auto cosSum = std::cos(thetaLeft) + std::cos(thetaRight);
  const auto theta = std::atan2(sinSum, cosSum);

  auto a00 = 0.0;
  auto a01 = 0.0;
  auto a11 = 0.0;
  auto b0 = 0.0;
  auto b1 = 0.0;

  const auto fillEquation = [&](const ad::localization::util::LineModel &scanLine,
                                const ad::localization::util::LineModel &mapLine) {
    const auto alphaMap = ad::localization::util::normalizeAngle(scanLine.alpha + theta);
    const auto normalX = std::cos(alphaMap);
    const auto normalY = std::sin(alphaMap);
    const auto rhs = mapLine.rho - scanLine.rho;

    a00 += normalX * normalX;
    a01 += normalX * normalY;
    a11 += normalY * normalY;
    b0 += normalX * rhs;
    b1 += normalY * rhs;
  };

  fillEquation(scanLeft, mapLeft);
  fillEquation(scanRight, mapRight);

  const auto det = (a00 * a11) - (a01 * a01);
  if (std::abs(det) < kSolveEpsilon) {
    return std::nullopt;
  }

  const auto x = ((a11 * b0) - (a01 * b1)) / det;
  const auto y = ((a00 * b1) - (a01 * b0)) / det;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta)) {
    return std::nullopt;
  }

  return ad::types::Pose{.x = x, .y = y, .theta = theta};
}

auto selectInliers(const ad::types::Pose &hypothesisPose,
                   const std::vector<CandidatePair> &candidates,
                   const std::vector<ad::localization::util::MapLine> &scanLines,
                   const std::vector<ad::localization::util::MapLine> &mapLines,
                   const ad::localization::HoughRansacObservationModelConfig &config)
    -> std::vector<MatchedPair> {
  const auto rhoThreshold = config.houghObservation.maxAssociationDistance;
  const auto angleThreshold =
      std::max(kDefaultInlierAngleThreshold, config.houghObservation.hough.mergeTheta * 2.0);

  struct PerScanBest {
    MatchedPair pair;
  };

  auto perScanBest = std::vector<std::optional<PerScanBest>>(scanLines.size());

  for (const auto &candidate : candidates) {
    const auto &scanLine = scanLines[candidate.scanLineIndex];
    const auto &mapLine = mapLines[candidate.mapLineIndex];

    auto transformed = transformLocalLineToMap(scanLine.model, hypothesisPose);
    const auto angleResidual =
        std::abs(ad::localization::util::normalizeAngle(transformed.alpha - mapLine.model.alpha));
    if (angleResidual > angleThreshold) {
      continue;
    }

    const auto rhoResidual = std::abs(transformed.rho - mapLine.model.rho);
    if (rhoResidual > rhoThreshold) {
      continue;
    }

    const auto score = angleResidual + (rhoResidual / std::max(rhoThreshold, 1e-6));
    const auto matched = MatchedPair{.scanLineIndex = candidate.scanLineIndex,
                                     .mapLineIndex = candidate.mapLineIndex,
                                     .angleResidual = angleResidual,
                                     .rhoResidual = rhoResidual,
                                     .score = score};

    auto &slot = perScanBest[candidate.scanLineIndex];
    if (!slot || matched.score < slot->pair.score) {
      slot = PerScanBest{.pair = matched};
    }
  }

  auto uniqueByScan = std::vector<MatchedPair>{};
  uniqueByScan.reserve(scanLines.size());
  for (const auto &slot : perScanBest) {
    if (slot) {
      uniqueByScan.push_back(slot->pair);
    }
  }

  std::sort(uniqueByScan.begin(), uniqueByScan.end(),
            [](const auto &left, const auto &right) -> bool { return left.score < right.score; });

  auto selected = std::vector<MatchedPair>{};
  auto mapUsed = std::vector<bool>(mapLines.size(), false);
  for (const auto &pair : uniqueByScan) {
    if (mapUsed[pair.mapLineIndex]) {
      continue;
    }
    mapUsed[pair.mapLineIndex] = true;
    selected.push_back(pair);
  }

  return selected;
}

auto hasOrientationDiversity(const std::vector<MatchedPair> &pairs,
                             const std::vector<ad::localization::util::MapLine> &mapLines) -> bool {
  if (pairs.size() < 2U) {
    return false;
  }

  auto sumCos = 0.0;
  auto sumSin = 0.0;
  for (const auto &pair : pairs) {
    const auto alpha = mapLines[pair.mapLineIndex].model.alpha;
    sumCos += std::cos(2.0 * alpha);
    sumSin += std::sin(2.0 * alpha);
  }

  const auto count = static_cast<double>(pairs.size());
  const auto concentration = std::hypot(sumCos, sumSin) / count;
  const auto diversity = 1.0 - concentration;
  return diversity >= kMinOrientationDiversity;
}

auto selectFallbackInliers(const ad::types::Pose &predictedPose,
                           const std::vector<CandidatePair> &candidates,
                           const std::vector<ad::localization::util::MapLine> &scanLines,
                           const std::vector<ad::localization::util::MapLine> &mapLines,
                           const ad::localization::HoughRansacObservationModelConfig &config)
    -> std::vector<MatchedPair> {
  auto fallback = selectInliers(predictedPose, candidates, scanLines, mapLines, config);
  if (fallback.empty()) {
    return fallback;
  }

  std::sort(fallback.begin(), fallback.end(),
            [](const auto &left, const auto &right) -> bool { return left.score < right.score; });

  const auto upperBound =
      std::max(config.houghObservation.minObservations, config.ransac.minInliers);
  if (fallback.size() > upperBound) {
    fallback.resize(upperBound);
  }

  return fallback;
}

auto fillObservationNoise(ad::localization::util::LineObservation &observation,
                          const ad::localization::HoughObservationModelConfig &config,
                          double residualRho, double residualAlpha) -> void {
  const auto baseRangeVar = config.measurementNoiseRange * config.measurementNoiseRange;
  const auto baseAngleVar = config.measurementNoiseAngle * config.measurementNoiseAngle;
  const auto pointScale = std::max(1.0, kReferencePoints / kReferencePoints);
  const auto minRangeVar = baseRangeVar * kMinRangeVarianceFactor;
  const auto minAngleVar = baseAngleVar * kMinAngleVarianceFactor;
  const auto mseLike =
      (residualRho * residualRho) + (residualAlpha * residualAlpha * kAngleMseScale);

  observation.rangeVariance = std::max(minRangeVar, (baseRangeVar * pointScale) + mseLike);
  observation.angleVariance =
      std::max(minAngleVar, (baseAngleVar * pointScale) + (mseLike * kAngleMseScale));
}

auto buildObservations(const std::vector<ad::localization::util::MapLine> &scanLines,
                       const std::vector<ad::localization::util::MapLine> &mapLines,
                       const std::vector<CandidatePair> &candidates,
                       const ad::types::Pose &predictedPose,
                       const ad::localization::HoughRansacObservationModelConfig &config,
                       const Mat3 &covariance) -> ObservationSummary {
  ObservationSummary summary{};

  if (candidates.size() < 2U) {
    return summary;
  }

  auto generator =
      std::mt19937(buildRansacSeed(predictedPose, scanLines.size(), candidates.size()));
  auto bestInliers = std::vector<MatchedPair>{};

  for (int iteration = 0; iteration < config.ransac.maxIterations; ++iteration) {
    const auto sample = drawSamplePair(generator, candidates.size());
    if (!sample) {
      continue;
    }

    const auto &left = candidates[sample->first];
    const auto &right = candidates[sample->second];
    const auto hypothesisPose = estimatePoseFromTwoPairs(left, right, scanLines, mapLines);
    if (!hypothesisPose) {
      continue;
    }

    auto inliers = selectInliers(*hypothesisPose, candidates, scanLines, mapLines, config);
    if (inliers.size() < config.ransac.minInliers) {
      continue;
    }

    const auto inlierRatio = static_cast<double>(inliers.size()) /
                             static_cast<double>(std::max<std::size_t>(1U, scanLines.size()));
    if (inlierRatio < config.ransac.minInlierRatio) {
      continue;
    }

    if (!hasOrientationDiversity(inliers, mapLines)) {
      continue;
    }

    if (inliers.size() > bestInliers.size()) {
      bestInliers = std::move(inliers);
    }
  }

  if (bestInliers.empty()) {
    bestInliers = selectFallbackInliers(predictedPose, candidates, scanLines, mapLines, config);
    if (bestInliers.size() < config.houghObservation.minObservations) {
      return summary;
    }
  }

  summary.observations.reserve(bestInliers.size());
  for (const auto &pair : bestInliers) {
    const auto &mapLine = mapLines[pair.mapLineIndex];
    const auto &scanLine = scanLines[pair.scanLineIndex];

    auto observation = ad::localization::util::makeExpectedLine(mapLine.model, predictedPose);
    observation.observed = scanLine.model;
    fillObservationNoise(observation, config.houghObservation, pair.rhoResidual,
                         pair.angleResidual);

    ++summary.candidates;
    if (!ad::localization::util::gateLineObservation(
            observation,
            ad::localization::util::ObservationGateConfig{
                .covariance = covariance, .threshold = config.houghObservation.gateThreshold})) {
      continue;
    }

    ++summary.gatePassed;
    summary.observations.push_back(observation);
  }

  return summary;
}

auto buildMeasurementData(const std::vector<ad::localization::util::LineObservation> &observations,
                          double score) -> ad::localization::ObservationUpdateInput {
  ad::localization::ObservationUpdateInput data{};
  const auto measurementCount = observations.size() * 2U;
  const auto size = static_cast<Eigen::Index>(measurementCount);
  data.residual = Eigen::VectorXd::Zero(size);
  data.measurementMatrix = Eigen::MatrixXd::Zero(size, 3);
  data.measurementNoise = Eigen::MatrixXd::Zero(size, size);
  data.score = score;

  for (std::size_t index = 0; index < observations.size(); ++index) {
    const auto &obs = observations[index];
    const auto row = static_cast<Eigen::Index>(index * 2U);

    const auto residualRho = obs.observed.rho - obs.expected.rho;
    const auto residualAlpha =
        ad::localization::util::normalizeAngle(obs.observed.alpha - obs.expected.alpha);
    data.residual(row) = residualRho;
    data.residual(row + 1) = residualAlpha;

    const auto hRhoX = -obs.rhoSign * obs.nx;
    const auto hRhoY = -obs.rhoSign * obs.ny;
    const auto hAlphaTheta = -1.0;

    data.measurementMatrix(row, 0) = hRhoX;
    data.measurementMatrix(row, 1) = hRhoY;
    data.measurementMatrix(row, 2) = 0.0;

    data.measurementMatrix(row + 1, 0) = 0.0;
    data.measurementMatrix(row + 1, 1) = 0.0;
    data.measurementMatrix(row + 1, 2) = hAlphaTheta;

    data.measurementNoise(row, row) = obs.rangeVariance;
    data.measurementNoise(row + 1, row + 1) = obs.angleVariance;
  }

  return data;
}

} // namespace

namespace ad::localization {

HoughRansacObservationModel::HoughRansacObservationModel(std::vector<util::MapLine> mapLines,
                                                         util::MapSignature signature,
                                                         HoughRansacObservationModelConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)) {}

auto HoughRansacObservationModel::create(const types::MapData &map,
                                         HoughRansacObservationModelConfig config)
    -> Result<std::unique_ptr<HoughRansacObservationModel>> {
  const auto signature = util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = hough::extractMapLinesFromMap(map, config.houghObservation.hough);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto observationModel =
      std::make_unique<HoughRansacObservationModel>(std::move(*mapLines), *signature, config);
  return {std::move(observationModel)};
}

auto HoughRansacObservationModel::buildUpdateInput(
    const types::LidarScan &scan, const types::MapData &map, const types::Pose &predictedPose,
    const CovarianceMatrix &predictedCovariance) const
    -> Result<std::optional<ObservationUpdateInput>> {
  if (!util::signatureMatches(mapSignature_, map)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map does not match precomputed line features."});
  }

  if (scan.ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan has no ranges."});
  }

  const auto scanPoints = collectScanPoints(scan);
  if (scanPoints.empty()) {
    return {std::nullopt};
  }

  const auto scanLines = hough::extractLinesFromPoints(scanPoints, config_.houghObservation.hough);
  if (!scanLines) {
    return {std::nullopt};
  }

  const auto candidates = buildCandidatePairs(*scanLines, mapLines_, predictedPose, config_);
  const auto summary = buildObservations(*scanLines, mapLines_, candidates, predictedPose, config_,
                                         predictedCovariance);

  if (summary.observations.size() < config_.houghObservation.minObservations) {
    return {std::nullopt};
  }

  const auto score = summary.candidates > 0 ? static_cast<double>(summary.gatePassed) /
                                                  static_cast<double>(summary.candidates)
                                            : 0.0;
  return {buildMeasurementData(summary.observations, score)};
}

} // namespace ad::localization

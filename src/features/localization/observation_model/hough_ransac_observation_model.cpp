#include "hough_ransac_observation_model.hpp"

#include "hough_line_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace {

constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;
constexpr double kMinSolveDeterminant = 1e-8;

using Mat3 = ad::localization::CovarianceMatrix;

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

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

auto buildRansacSeed(const ad::types::Pose &pose, std::size_t candidateCount) -> std::uint32_t {
  const auto poseSeed = static_cast<std::uint32_t>(
      std::llround((std::abs(pose.x) + std::abs(pose.y) + std::abs(pose.theta)) * 1000.0));
  return static_cast<std::uint32_t>(candidateCount * 2654435761U) ^ poseSeed;
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

auto transformLocalLineToMap(const ad::localization::util::LineModel &localLine,
                             const ad::types::Pose &pose) -> ad::localization::util::LineModel {
  const auto alphaMap = ad::localization::util::normalizeAngle(localLine.alpha + pose.theta);
  const auto rhoMap = localLine.rho + (pose.x * std::cos(alphaMap)) + (pose.y * std::sin(alphaMap));
  return ad::localization::util::toLineModel(
      ad::localization::util::LineModel{.rho = rhoMap, .alpha = alphaMap});
}

auto buildCandidatePairs(const std::vector<ad::localization::util::MapLine> &scanLines,
                         const std::vector<ad::localization::util::MapLine> &mapLines,
                         const ad::types::Pose &predictedPose,
                         const ad::localization::HoughRansacObservationModelConfig &config)
    -> std::vector<CandidatePair> {
  auto candidates = std::vector<CandidatePair>{};
  const auto angleGate = std::max(0.2, config.houghObservation.hough.mergeTheta * 2.0);
  const auto rhoGate = std::max(0.4, config.houghObservation.maxAssociationDistance * 2.0);

  for (std::size_t scanIndex = 0; scanIndex < scanLines.size(); ++scanIndex) {
    const auto predictedMapLine =
        transformLocalLineToMap(scanLines[scanIndex].model, predictedPose);

    for (std::size_t mapIndex = 0; mapIndex < mapLines.size(); ++mapIndex) {
      const auto angleResidual = std::abs(ad::localization::util::normalizeAngle(
          predictedMapLine.alpha - mapLines[mapIndex].model.alpha));
      if (angleResidual > angleGate) {
        continue;
      }

      const auto rhoResidual = std::abs(predictedMapLine.rho - mapLines[mapIndex].model.rho);
      if (rhoResidual > rhoGate) {
        continue;
      }

      const auto coarseScore = angleResidual + (rhoResidual / rhoGate);
      candidates.push_back(CandidatePair{
          .scanLineIndex = scanIndex, .mapLineIndex = mapIndex, .coarseScore = coarseScore});
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const auto &left, const auto &right) {
    return left.coarseScore < right.coarseScore;
  });
  return candidates;
}

auto sampleTwoDistinct(std::mt19937 &generator, std::size_t count)
    -> std::optional<std::pair<std::size_t, std::size_t>> {
  if (count < 2U) {
    return std::nullopt;
  }

  std::uniform_int_distribution<std::size_t> distribution(0U, count - 1U);
  const auto index0 = distribution(generator);
  auto index1 = distribution(generator);
  if (index0 == index1) {
    return std::nullopt;
  }
  return std::pair<std::size_t, std::size_t>{index0, index1};
}

auto estimatePoseFromTwoPairs(const CandidatePair &pair0, const CandidatePair &pair1,
                              const std::vector<ad::localization::util::MapLine> &scanLines,
                              const std::vector<ad::localization::util::MapLine> &mapLines)
    -> std::optional<ad::types::Pose> {
  if (pair0.scanLineIndex == pair1.scanLineIndex || pair0.mapLineIndex == pair1.mapLineIndex) {
    return std::nullopt;
  }

  const auto &scan0 = scanLines[pair0.scanLineIndex].model;
  const auto &scan1 = scanLines[pair1.scanLineIndex].model;
  const auto &map0 = mapLines[pair0.mapLineIndex].model;
  const auto &map1 = mapLines[pair1.mapLineIndex].model;

  const auto theta0 = ad::localization::util::normalizeAngle(map0.alpha - scan0.alpha);
  const auto theta1 = ad::localization::util::normalizeAngle(map1.alpha - scan1.alpha);
  const auto theta =
      std::atan2(std::sin(theta0) + std::sin(theta1), std::cos(theta0) + std::cos(theta1));

  auto matrix00 = 0.0;
  auto matrix01 = 0.0;
  auto matrix11 = 0.0;
  auto rhs0 = 0.0;
  auto rhs1 = 0.0;

  const auto addEquation = [&](const ad::localization::util::LineModel &scanLine,
                               const ad::localization::util::LineModel &mapLine) {
    const auto alpha = ad::localization::util::normalizeAngle(scanLine.alpha + theta);
    const auto normalX = std::cos(alpha);
    const auto normalY = std::sin(alpha);
    const auto rhs = mapLine.rho - scanLine.rho;

    matrix00 += normalX * normalX;
    matrix01 += normalX * normalY;
    matrix11 += normalY * normalY;
    rhs0 += normalX * rhs;
    rhs1 += normalY * rhs;
  };

  addEquation(scan0, map0);
  addEquation(scan1, map1);

  const auto determinant = (matrix00 * matrix11) - (matrix01 * matrix01);
  if (std::abs(determinant) < kMinSolveDeterminant) {
    return std::nullopt;
  }

  const auto poseX = ((matrix11 * rhs0) - (matrix01 * rhs1)) / determinant;
  const auto poseY = ((matrix00 * rhs1) - (matrix01 * rhs0)) / determinant;
  if (!std::isfinite(poseX) || !std::isfinite(poseY) || !std::isfinite(theta)) {
    return std::nullopt;
  }

  return ad::types::Pose{.x = poseX, .y = poseY, .theta = theta};
}

auto collectInliers(const ad::types::Pose &hypothesisPose,
                    const std::vector<CandidatePair> &candidates,
                    const std::vector<ad::localization::util::MapLine> &scanLines,
                    const std::vector<ad::localization::util::MapLine> &mapLines,
                    const ad::localization::HoughRansacObservationModelConfig &config)
    -> std::vector<MatchedPair> {
  const auto angleThreshold = std::max(0.16, config.houghObservation.hough.mergeTheta * 2.0);
  const auto rhoThreshold = std::max(config.houghObservation.maxAssociationDistance, 1e-6);

  auto matched = std::vector<MatchedPair>{};
  matched.reserve(candidates.size());

  for (const auto &candidate : candidates) {
    const auto transformed =
        transformLocalLineToMap(scanLines[candidate.scanLineIndex].model, hypothesisPose);

    const auto angleResidual = std::abs(ad::localization::util::normalizeAngle(
        transformed.alpha - mapLines[candidate.mapLineIndex].model.alpha));
    if (angleResidual > angleThreshold) {
      continue;
    }

    const auto rhoResidual = std::abs(transformed.rho - mapLines[candidate.mapLineIndex].model.rho);
    if (rhoResidual > rhoThreshold) {
      continue;
    }

    const auto score = angleResidual + (rhoResidual / rhoThreshold);
    matched.push_back(MatchedPair{.scanLineIndex = candidate.scanLineIndex,
                                  .mapLineIndex = candidate.mapLineIndex,
                                  .angleResidual = angleResidual,
                                  .rhoResidual = rhoResidual,
                                  .score = score});
  }

  std::sort(matched.begin(), matched.end(),
            [](const auto &left, const auto &right) { return left.score < right.score; });

  auto selected = std::vector<MatchedPair>{};
  auto scanUsed = std::vector<bool>(scanLines.size(), false);
  auto mapUsed = std::vector<bool>(mapLines.size(), false);
  for (const auto &pair : matched) {
    if (scanUsed[pair.scanLineIndex] || mapUsed[pair.mapLineIndex]) {
      continue;
    }
    scanUsed[pair.scanLineIndex] = true;
    mapUsed[pair.mapLineIndex] = true;
    selected.push_back(pair);
  }

  return selected;
}

auto runPairRansac(const std::vector<CandidatePair> &candidates,
                   const std::vector<ad::localization::util::MapLine> &scanLines,
                   const std::vector<ad::localization::util::MapLine> &mapLines,
                   const ad::types::Pose &predictedPose,
                   const ad::localization::HoughRansacObservationModelConfig &config)
    -> std::vector<MatchedPair> {
  if (candidates.size() < 2U || config.ransac.maxIterations <= 0) {
    return {};
  }

  auto generator = std::mt19937(buildRansacSeed(predictedPose, candidates.size()));
  auto bestInliers = std::vector<MatchedPair>{};

  for (int iteration = 0; iteration < config.ransac.maxIterations; ++iteration) {
    const auto sample = sampleTwoDistinct(generator, candidates.size());
    if (!sample) {
      continue;
    }

    const auto poseHypothesis = estimatePoseFromTwoPairs(
        candidates[sample->first], candidates[sample->second], scanLines, mapLines);
    if (!poseHypothesis) {
      continue;
    }

    auto inliers = collectInliers(*poseHypothesis, candidates, scanLines, mapLines, config);
    if (inliers.size() < config.ransac.minInliers) {
      continue;
    }

    const auto inlierRatio = static_cast<double>(inliers.size()) /
                             static_cast<double>(std::max<std::size_t>(1U, scanLines.size()));
    if (inlierRatio < config.ransac.minInlierRatio) {
      continue;
    }

    if (inliers.size() > bestInliers.size()) {
      bestInliers = std::move(inliers);
    }
  }

  return bestInliers;
}

auto fillObservationNoise(ad::localization::util::LineObservation &observation,
                          const ad::localization::HoughObservationModelConfig &config,
                          double angleResidual, double rhoResidual) -> void {
  const auto baseRangeVar = config.measurementNoiseRange * config.measurementNoiseRange;
  const auto baseAngleVar = config.measurementNoiseAngle * config.measurementNoiseAngle;
  const auto minRangeVar = baseRangeVar * kMinRangeVarianceFactor;
  const auto minAngleVar = baseAngleVar * kMinAngleVarianceFactor;
  const auto scoreMse =
      (rhoResidual * rhoResidual) + (angleResidual * angleResidual * kAngleMseScale);
  const auto scale = std::max(1.0, kReferencePoints / kReferencePoints);

  observation.rangeVariance = std::max(minRangeVar, (baseRangeVar * scale) + scoreMse);
  observation.angleVariance =
      std::max(minAngleVar, (baseAngleVar * scale) + (scoreMse * kAngleMseScale));
}

auto buildObservations(const std::vector<MatchedPair> &inliers,
                       const std::vector<ad::localization::util::MapLine> &scanLines,
                       const std::vector<ad::localization::util::MapLine> &mapLines,
                       const ad::types::Pose &predictedPose,
                       const ad::localization::HoughRansacObservationModelConfig &config,
                       const Mat3 &covariance) -> ObservationSummary {
  auto summary = ObservationSummary{};
  summary.observations.reserve(inliers.size());

  for (const auto &pair : inliers) {
    const auto &mapLine = mapLines[pair.mapLineIndex];
    const auto &scanLine = scanLines[pair.scanLineIndex];

    auto observation = ad::localization::util::makeExpectedLine(mapLine.model, predictedPose);
    observation.observed = scanLine.model;
    fillObservationNoise(observation, config.houghObservation, pair.angleResidual,
                         pair.rhoResidual);

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
  const auto inliers = runPairRansac(candidates, *scanLines, mapLines_, predictedPose, config_);
  const auto summary = buildObservations(inliers, *scanLines, mapLines_, predictedPose, config_,
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

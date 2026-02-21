#include "hough_ransac_observation_model.hpp"

#include "hough_line_extractor.hpp"
#include "observation_model_common.hpp"
#include "ransac_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

constexpr double kCandidateAngleGateMin = 0.2;
constexpr double kCandidateRhoGateMin = 0.4;
constexpr double kInlierAngleThresholdMin = 0.16;

using Mat3 = ad::localization::CovarianceMatrix;

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
    -> std::vector<ad::localization::ransac::LinePairCandidate> {
  auto candidates = std::vector<ad::localization::ransac::LinePairCandidate>{};
  const auto angleGate =
      std::max(kCandidateAngleGateMin, config.houghObservation.hough.mergeTheta * 2.0);
  const auto rhoGate =
      std::max(kCandidateRhoGateMin, config.houghObservation.maxAssociationDistance * 2.0);

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

      candidates.push_back(
          ad::localization::ransac::LinePairCandidate{.scanLineIndex = scanIndex,
                                                      .mapLineIndex = mapIndex,
                                                      .scanLine = scanLines[scanIndex].model,
                                                      .mapLine = mapLines[mapIndex].model});
    }
  }

  return candidates;
}
auto buildObservations(const std::vector<ad::localization::ransac::LinePairMatch> &inliers,
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
    ad::localization::observation_model_common::applyObservationNoiseFromResidual(
        observation, config.houghObservation, pair.angleResidual, pair.rhoResidual);

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
  const auto inliers = ad::localization::ransac::runLinePairRansac(
      candidates,
      ad::localization::ransac::LinePairRansacConfig{
          .maxIterations = config_.ransac.maxIterations,
          .minInliers = config_.ransac.minInliers,
          .minInlierRatio = config_.ransac.minInlierRatio,
          .inlierAngleThreshold =
              std::max(kInlierAngleThresholdMin, config_.houghObservation.hough.mergeTheta * 2.0),
          .inlierRhoThreshold = std::max(config_.houghObservation.maxAssociationDistance, 1e-6),
          .lineCountForRatio = scanLines->size()},
      buildRansacSeed(predictedPose, candidates.size()));
  const auto summary = buildObservations(inliers, *scanLines, mapLines_, predictedPose, config_,
                                         predictedCovariance);

  if (summary.observations.size() < config_.houghObservation.minObservations) {
    return {std::nullopt};
  }

  const auto score = summary.candidates > 0 ? static_cast<double>(summary.gatePassed) /
                                                  static_cast<double>(summary.candidates)
                                            : 0.0;
  return {ad::localization::observation_model_common::buildMeasurementData(summary.observations,
                                                                           score)};
}

} // namespace ad::localization

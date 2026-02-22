#include "ransac_line_association_model.hpp"

#include "line_association_engine.hpp"
#include "line_extractor.hpp"
#include "line_observation_builder.hpp"
#include "observation_diagnostics_sink.hpp"
#include "ransac_core.hpp"
#include "scan_line_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr double kCandidateAngleGateMin = 0.08;
constexpr double kCandidateRhoGateMin = 0.2;
constexpr double kInlierAngleThresholdMin = 0.08;

using Mat3 = ad::localization::CovarianceMatrix;

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

using UpdateDebugRecord =
    ad::localization::observation_diagnostics_sink::RansacLineAssociationUpdateRecord;

auto buildRansacSeed(const ad::types::Pose &pose, std::size_t candidateCount) -> std::uint32_t {
  const auto poseSeed = static_cast<std::uint32_t>(
      std::llround((std::abs(pose.x) + std::abs(pose.y) + std::abs(pose.theta)) * 1000.0));
  return static_cast<std::uint32_t>(candidateCount * 2654435761U) ^ poseSeed;
}

auto buildObservations(const std::vector<ad::localization::ransac::LinePairMatch> &inliers,
                       const std::vector<ad::localization::util::MapLine> &scanLines,
                       const std::vector<ad::localization::util::MapLine> &mapLines,
                       const ad::types::Pose &predictedPose,
                       const ad::localization::RansacLineAssociationModelConfig &config,
                       const Mat3 &covariance) -> ObservationSummary {
  auto summary = ObservationSummary{};
  summary.observations.reserve(inliers.size());

  for (const auto &pair : inliers) {
    const auto &mapLine = mapLines[pair.mapLineIndex];
    const auto &scanLine = scanLines[pair.scanLineIndex];

    ++summary.candidates;
    const auto observation =
        ad::localization::line_observation_builder::buildGatedObservationFromResidual(
            mapLine.model, scanLine.model, predictedPose, pair.angleResidual, pair.rhoResidual,
            ad::localization::line_observation_builder::ObservationBuildConfig{
                .measurementNoiseRange = config.baseObservation.measurementNoiseRange,
                .measurementNoiseAngle = config.baseObservation.measurementNoiseAngle,
                .gateThreshold = config.baseObservation.gateThreshold},
            covariance);
    if (!observation) {
      continue;
    }

    ++summary.gatePassed;
    summary.observations.push_back(*observation);
  }

  return summary;
}

} // namespace

namespace ad::localization {

RansacLineAssociationModel::RansacLineAssociationModel(std::vector<util::MapLine> mapLines,
                                                       util::MapSignature signature,
                                                       RansacLineAssociationModelConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)) {}

auto RansacLineAssociationModel::create(const types::MapData &map,
                                        RansacLineAssociationModelConfig config)
    -> Result<std::unique_ptr<RansacLineAssociationModel>> {
  const auto signature = util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = line_extractor::extractMapLinesFromMap(
      map,
      MapLineExtractionConfig{.maxLines = config.ransacLineExtraction.maxLines,
                              .minSegmentLength = config.ransacLineExtraction.minSegmentLength});
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto observationModel =
      std::make_unique<RansacLineAssociationModel>(std::move(*mapLines), *signature, config);
  return {std::move(observationModel)};
}

auto RansacLineAssociationModel::buildUpdateInput(const types::LidarScan &scan,
                                                  const types::MapData &map,
                                                  const types::Pose &predictedPose,
                                                  const CovarianceMatrix &predictedCovariance) const
    -> Result<std::optional<ObservationUpdateInput>> {
  if (!util::signatureMatches(mapSignature_, map)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map does not match precomputed line features."});
  }

  if (scan.ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan has no ranges."});
  }

  auto debugRecord = UpdateDebugRecord{};
  const auto diagnosticsSink =
      ad::localization::observation_diagnostics_sink::defaultRansacLineAssociationDiagnosticsSink();
  debugRecord.minObservations = config_.baseObservation.minObservations;

  const auto scanPoints = scan_line_extractor::collectScanPoints(scan);
  debugRecord.scanPointCount = scanPoints.size();
  if (scanPoints.empty()) {
    debugRecord.reason = "scan_points_empty";
    diagnosticsSink->append(debugRecord);
    return {std::nullopt};
  }

  const auto scanLines = scan_line_extractor::extractLinesFromPointsRansac(
      scanPoints, scan_line_extractor::RansacScanLineExtractionConfig{
                      .lineExtraction = config_.ransacLineExtraction, .ransac = config_.ransac});
  if (!scanLines) {
    debugRecord.reason = "scan_line_extract_failed";
    diagnosticsSink->append(debugRecord);
    return {std::nullopt};
  }

  debugRecord.scanLineCount = scanLines->size();

  const auto candidates = line_association_engine::buildCandidatePairs(
      *scanLines, mapLines_, predictedPose,
      line_association_engine::CandidatePairBuildConfig{
          .candidateAngleGateMin = kCandidateAngleGateMin,
          .candidateRhoGateMin = kCandidateRhoGateMin,
          .mergeTheta = config_.ransacLineExtraction.mergeTheta,
          .maxAssociationDistance = config_.baseObservation.maxAssociationDistance,
          .segmentMargin = config_.baseObservation.segmentMargin});
  debugRecord.candidatePairCount = candidates.size();
  const auto ratioLineCount =
      std::max<std::size_t>(1U, line_association_engine::countDistinctScanLines(candidates));

  const auto ransacResult = ad::localization::ransac::runLinePairRansacWithDiagnostics(
      candidates,
      ad::localization::ransac::LinePairRansacConfig{
          .maxIterations = config_.ransac.maxIterations,
          .minInliers = config_.ransac.minInliers,
          .minInlierRatio = config_.ransac.minInlierRatio,
          .inlierAngleThreshold =
              std::max(kInlierAngleThresholdMin, config_.ransacLineExtraction.mergeTheta * 2.0),
          .inlierRhoThreshold = std::max(config_.baseObservation.maxAssociationDistance, 1e-6),
          .lineCountForRatio = ratioLineCount,
          .usePosePrior = true,
          .priorPoseX = predictedPose.x,
          .priorPoseY = predictedPose.y,
          .priorPoseTheta = predictedPose.theta,
          .maxTranslationDelta = std::max(
              1.8, 8.0 * std::sqrt(std::max(predictedCovariance(0, 0), predictedCovariance(1, 1)))),
          .maxRotationDelta =
              std::max(1.0, 8.0 * std::sqrt(std::max(predictedCovariance(2, 2), 1e-9))),
          .maxMeanResidual = 1.2},
      buildRansacSeed(predictedPose, candidates.size()));

  debugRecord.ransacDiagnostics = ransacResult.diagnostics;
  debugRecord.ransacInlierCount = ransacResult.inliers.size();

  const auto summary = buildObservations(ransacResult.inliers, *scanLines, mapLines_, predictedPose,
                                         config_, predictedCovariance);
  debugRecord.observationCandidates = summary.candidates;
  debugRecord.observationGatePassed = summary.gatePassed;
  debugRecord.finalObservationCount = summary.observations.size();

  if (summary.observations.size() < config_.baseObservation.minObservations) {
    if (!scanLines->empty() && candidates.empty()) {
      debugRecord.reason = "no_candidate_pairs";
    } else if (!ransacResult.diagnostics.configurationValid) {
      debugRecord.reason = "ransac_invalid_config";
    } else if (!ransacResult.inliers.empty() && summary.gatePassed == 0) {
      debugRecord.reason = "all_gate_rejected";
    } else if (ransacResult.inliers.empty()) {
      debugRecord.reason = "ransac_no_inliers";
    } else {
      debugRecord.reason = "below_min_observations";
    }
    diagnosticsSink->append(debugRecord);
    return {std::nullopt};
  }

  const auto score = summary.candidates > 0 ? static_cast<double>(summary.gatePassed) /
                                                  static_cast<double>(summary.candidates)
                                            : 0.0;
  debugRecord.reason = "success";
  diagnosticsSink->append(debugRecord);
  return {ad::localization::util::buildMeasurementData(summary.observations, score)};
}

} // namespace ad::localization

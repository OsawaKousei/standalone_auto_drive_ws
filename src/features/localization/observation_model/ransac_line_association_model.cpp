#include "ransac_line_association_model.hpp"

#include "observation_model_common.hpp"
#include "ransac_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr double kCandidateAngleGateMin = 0.08;
constexpr double kCandidateRhoGateMin = 0.2;
constexpr double kInlierAngleThresholdMin = 0.08;
constexpr double kLineExtractionDistanceMin = 1e-6;
constexpr double kLineExtractionSpanMin = 1e-3;

using Mat3 = ad::localization::CovarianceMatrix;

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

struct UpdateDebugRecord {
  std::size_t scanPointCount = 0U;
  std::size_t scanLineCount = 0U;
  std::size_t candidatePairCount = 0U;
  std::size_t ransacInlierCount = 0U;
  int observationCandidates = 0;
  int observationGatePassed = 0;
  std::size_t finalObservationCount = 0U;
  std::size_t minObservations = 0U;
  ad::localization::ransac::LinePairRansacDiagnostics ransacDiagnostics{};
  std::string reason;
};

auto appendUpdateDebugCsv(const UpdateDebugRecord &record) -> void {
  static std::mutex mutex;
  const auto lock = std::lock_guard<std::mutex>{mutex};

  std::error_code error;
  std::filesystem::create_directories("test/localization/logs", error);

  constexpr auto path = "test/localization/logs/ransac_line_association_update_debug.csv";
  const auto needHeader = !std::filesystem::exists(path);
  auto stream = std::ofstream(path, std::ios::app);
  if (!stream.is_open()) {
    return;
  }

  if (needHeader) {
    stream << "scan_point_count,scan_line_count,candidate_pair_count,ransac_inlier_count,"
              "obs_candidates,obs_gate_passed,final_observation_count,min_observations,"
              "reason,ransac_config_valid,ransac_iterations_requested,"
              "ransac_duplicate_sample_rejects,ransac_hypothesis_rejects,"
              "ransac_min_inlier_rejects,ransac_ratio_rejects,ransac_pose_prior_rejects,"
              "ransac_residual_rejects,ransac_accepted_hypotheses,"
              "ransac_best_inlier_count\n";
  }

  stream << record.scanPointCount << ',' << record.scanLineCount << ',' << record.candidatePairCount
         << ',' << record.ransacInlierCount << ',' << record.observationCandidates << ','
         << record.observationGatePassed << ',' << record.finalObservationCount << ','
         << record.minObservations << ',' << record.reason << ','
         << (record.ransacDiagnostics.configurationValid ? 1 : 0) << ','
         << record.ransacDiagnostics.iterationsRequested << ','
         << record.ransacDiagnostics.duplicateSampleRejects << ','
         << record.ransacDiagnostics.hypothesisRejects << ','
         << record.ransacDiagnostics.minInlierRejects << ','
         << record.ransacDiagnostics.ratioRejects << ','
         << record.ransacDiagnostics.posePriorRejects << ','
         << record.ransacDiagnostics.residualRejects << ','
         << record.ransacDiagnostics.acceptedHypotheses << ','
         << record.ransacDiagnostics.bestInlierCount << '\n';
}

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

auto isTooCloseToExisting(const ad::localization::util::LineModel &candidate,
                          const std::vector<ad::localization::util::MapLine> &lines,
                          const ad::localization::RansacLineExtractionConfig &lineExtraction)
    -> bool {
  return std::any_of(lines.begin(), lines.end(), [&](const auto &existing) {
    const auto rhoDiff = std::abs(existing.model.rho - candidate.rho);
    const auto alphaDiff =
        std::abs(ad::localization::util::normalizeAngle(existing.model.alpha - candidate.alpha));
    return rhoDiff <= lineExtraction.mergeRho && alphaDiff <= lineExtraction.mergeTheta;
  });
}

auto buildMapLineFromInliers(const ad::localization::util::LineModel &model,
                             const std::vector<ad::types::Point> &points,
                             const std::vector<std::size_t> &inlierIndices)
    -> std::optional<ad::localization::util::MapLine> {
  if (inlierIndices.size() < 2U) {
    return std::nullopt;
  }

  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);
  const auto tangentX = -normalY;
  const auto tangentY = normalX;

  auto minProjection = std::numeric_limits<double>::infinity();
  auto maxProjection = -std::numeric_limits<double>::infinity();
  for (const auto index : inlierIndices) {
    const auto &point = points[index];
    const auto projection = (tangentX * point.x) + (tangentY * point.y);
    minProjection = std::min(minProjection, projection);
    maxProjection = std::max(maxProjection, projection);
  }

  if (!std::isfinite(minProjection) || !std::isfinite(maxProjection) ||
      (maxProjection - minProjection) < kLineExtractionSpanMin) {
    return std::nullopt;
  }

  const auto start = ad::types::Point{.x = (tangentX * minProjection) + (normalX * model.rho),
                                      .y = (tangentY * minProjection) + (normalY * model.rho)};
  const auto end = ad::types::Point{.x = (tangentX * maxProjection) + (normalX * model.rho),
                                    .y = (tangentY * maxProjection) + (normalY * model.rho)};

  const auto segmentDeltaX = end.x - start.x;
  const auto segmentDeltaY = end.y - start.y;
  const auto segmentLength = std::hypot(segmentDeltaX, segmentDeltaY);
  if (segmentLength < kLineExtractionSpanMin) {
    return std::nullopt;
  }

  const auto directionX = segmentDeltaX / segmentLength;
  const auto directionY = segmentDeltaY / segmentLength;
  auto projectionMinValue = (directionX * start.x) + (directionY * start.y);
  auto projectionMaxValue = (directionX * end.x) + (directionY * end.y);
  if (projectionMinValue > projectionMaxValue) {
    std::swap(projectionMinValue, projectionMaxValue);
  }

  return ad::localization::util::MapLine{ad::types::LineSegment{start, end},
                                         model,
                                         directionX,
                                         directionY,
                                         projectionMinValue,
                                         projectionMaxValue};
}

auto extractLinesFromPointsRansac(const std::vector<ad::types::Point> &points,
                                  const ad::localization::RansacLineAssociationModelConfig &config)
    -> ad::Result<std::vector<ad::localization::util::MapLine>> {
  if (points.size() < 2U) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::EmptyCollection, "Point set is too small for RANSAC."});
  }

  const auto maxLines = std::max(1, config.ransacLineExtraction.maxLines);
  const auto inlierDistance =
      std::max(kLineExtractionDistanceMin, config.ransacLineExtraction.inlierDistance);
  const auto minInlierSpan =
      std::max(kLineExtractionSpanMin, config.ransacLineExtraction.minSegmentLength);

  auto remainingPoints = points;
  auto lines = std::vector<ad::localization::util::MapLine>{};
  lines.reserve(static_cast<std::size_t>(maxLines));

  for (int lineIndex = 0; lineIndex < maxLines && remainingPoints.size() >= 2U; ++lineIndex) {
    const auto ratioDrivenMinInliers =
        static_cast<std::size_t>(std::ceil(0.02 * static_cast<double>(remainingPoints.size())));
    const auto requestedMinInliers = std::min<std::size_t>(
        config.ransac.minInliers, std::max<std::size_t>(2U, ratioDrivenMinInliers));
    const auto minInliers = std::max<std::size_t>(
        2U, std::min<std::size_t>(requestedMinInliers, remainingPoints.size()));
    const auto minInlierRatio = std::max(0.002, std::min(config.ransac.minInlierRatio, 0.08));

    const auto fit = ad::localization::ransac::fitLineToPoints(
        remainingPoints, ad::localization::ransac::PointLineRansacConfig{
                             .maxIterations = config.ransac.maxIterations,
                             .inlierDistance = inlierDistance,
                             .minInliers = minInliers,
                             .minInlierRatio = minInlierRatio,
                             .minInlierSpan = minInlierSpan});
    if (!fit) {
      break;
    }

    const auto line = buildMapLineFromInliers(fit->fit.model, remainingPoints, fit->inlierIndices);
    if (line && !isTooCloseToExisting(fit->fit.model, lines, config.ransacLineExtraction)) {
      lines.push_back(*line);
    }

    auto inlierMask = std::vector<bool>(remainingPoints.size(), false);
    for (const auto index : fit->inlierIndices) {
      inlierMask[index] = true;
    }

    auto nextPoints = std::vector<ad::types::Point>{};
    nextPoints.reserve(remainingPoints.size() - fit->inlierIndices.size());
    for (std::size_t index = 0; index < remainingPoints.size(); ++index) {
      if (!inlierMask[index]) {
        nextPoints.push_back(remainingPoints[index]);
      }
    }
    remainingPoints = std::move(nextPoints);
  }

  if (lines.empty()) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::EmptyCollection, "No line segments extracted by RANSAC."});
  }

  return lines;
}

auto extractMapLinesRansac(const ad::types::MapData &map,
                           const ad::localization::RansacLineAssociationModelConfig &config)
    -> ad::Result<std::vector<ad::localization::util::MapLine>> {
  if (!ad::localization::util::mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  const auto points = ad::localization::util::collectOccupiedPoints(map);
  if (points.empty()) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::EmptyCollection, "Map contains no occupied cells."});
  }

  return extractLinesFromPointsRansac(points, config);
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
                         const ad::localization::RansacLineAssociationModelConfig &config)
    -> std::vector<ad::localization::ransac::LinePairCandidate> {
  auto candidates = std::vector<ad::localization::ransac::LinePairCandidate>{};
  const auto angleGate =
      std::max(kCandidateAngleGateMin, config.ransacLineExtraction.mergeTheta * 2.0);
  const auto rhoGate =
      std::max(kCandidateRhoGateMin, config.baseObservation.maxAssociationDistance * 2.0);

  for (std::size_t scanIndex = 0; scanIndex < scanLines.size(); ++scanIndex) {
    const auto predictedMapLine =
        transformLocalLineToMap(scanLines[scanIndex].model, predictedPose);
    const auto scanStart = scanLines[scanIndex].segment.start;
    const auto scanEnd = scanLines[scanIndex].segment.end;
    const auto cosTheta = std::cos(predictedPose.theta);
    const auto sinTheta = std::sin(predictedPose.theta);
    const auto scanStartMap = ad::types::Point{
        .x = predictedPose.x + (cosTheta * scanStart.x) - (sinTheta * scanStart.y),
        .y = predictedPose.y + (sinTheta * scanStart.x) + (cosTheta * scanStart.y)};
    const auto scanEndMap =
        ad::types::Point{.x = predictedPose.x + (cosTheta * scanEnd.x) - (sinTheta * scanEnd.y),
                         .y = predictedPose.y + (sinTheta * scanEnd.x) + (cosTheta * scanEnd.y)};

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

      const auto scanProjection0 = (mapLines[mapIndex].directionX * scanStartMap.x) +
                                   (mapLines[mapIndex].directionY * scanStartMap.y);
      const auto scanProjection1 = (mapLines[mapIndex].directionX * scanEndMap.x) +
                                   (mapLines[mapIndex].directionY * scanEndMap.y);
      const auto scanProjectionMin = std::min(scanProjection0, scanProjection1);
      const auto scanProjectionMax = std::max(scanProjection0, scanProjection1);
      const auto mapProjectionMin =
          mapLines[mapIndex].minProjection - config.baseObservation.segmentMargin;
      const auto mapProjectionMax =
          mapLines[mapIndex].maxProjection + config.baseObservation.segmentMargin;
      const auto overlapMin = std::max(scanProjectionMin, mapProjectionMin);
      const auto overlapMax = std::min(scanProjectionMax, mapProjectionMax);
      if (overlapMax < overlapMin) {
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
                       const ad::localization::RansacLineAssociationModelConfig &config,
                       const Mat3 &covariance) -> ObservationSummary {
  auto summary = ObservationSummary{};
  summary.observations.reserve(inliers.size());

  for (const auto &pair : inliers) {
    const auto &mapLine = mapLines[pair.mapLineIndex];
    const auto &scanLine = scanLines[pair.scanLineIndex];

    auto observation = ad::localization::util::makeExpectedLine(mapLine.model, predictedPose);
    observation.observed = scanLine.model;
    ad::localization::observation_model_common::applyObservationNoiseFromResidual(
        observation, config.baseObservation, pair.angleResidual, pair.rhoResidual);

    ++summary.candidates;
    if (!ad::localization::util::gateLineObservation(
            observation,
            ad::localization::util::ObservationGateConfig{
                .covariance = covariance, .threshold = config.baseObservation.gateThreshold})) {
      continue;
    }

    ++summary.gatePassed;
    summary.observations.push_back(observation);
  }

  return summary;
}

auto countDistinctScanLines(
    const std::vector<ad::localization::ransac::LinePairCandidate> &candidates) -> std::size_t {
  auto scanIndices = std::vector<std::size_t>{};
  scanIndices.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    scanIndices.push_back(candidate.scanLineIndex);
  }
  std::sort(scanIndices.begin(), scanIndices.end());
  const auto uniqueEnd = std::unique(scanIndices.begin(), scanIndices.end());
  return static_cast<std::size_t>(std::distance(scanIndices.begin(), uniqueEnd));
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

  const auto mapLines = extractMapLinesRansac(map, config);
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
  debugRecord.minObservations = config_.baseObservation.minObservations;

  const auto scanPoints = collectScanPoints(scan);
  debugRecord.scanPointCount = scanPoints.size();
  if (scanPoints.empty()) {
    debugRecord.reason = "scan_points_empty";
    appendUpdateDebugCsv(debugRecord);
    return {std::nullopt};
  }

  const auto scanLines = extractLinesFromPointsRansac(scanPoints, config_);
  if (!scanLines) {
    debugRecord.reason = "scan_line_extract_failed";
    appendUpdateDebugCsv(debugRecord);
    return {std::nullopt};
  }

  debugRecord.scanLineCount = scanLines->size();

  const auto candidates = buildCandidatePairs(*scanLines, mapLines_, predictedPose, config_);
  debugRecord.candidatePairCount = candidates.size();
  const auto ratioLineCount = std::max<std::size_t>(1U, countDistinctScanLines(candidates));

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
    appendUpdateDebugCsv(debugRecord);
    return {std::nullopt};
  }

  const auto score = summary.candidates > 0 ? static_cast<double>(summary.gatePassed) /
                                                  static_cast<double>(summary.candidates)
                                            : 0.0;
  debugRecord.reason = "success";
  appendUpdateDebugCsv(debugRecord);
  return {ad::localization::observation_model_common::buildMeasurementData(summary.observations,
                                                                           score)};
}

} // namespace ad::localization

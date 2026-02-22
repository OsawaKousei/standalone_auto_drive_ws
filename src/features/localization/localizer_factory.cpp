#include "localizer_factory.hpp"

#include "i_observation_model.hpp"
#include "localizer/ekf_localizer.hpp"
#include "observation_model/ransac_line_association_model.hpp"
#include "observation_model/simple_line_association_model.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace ad::localization {

namespace {

[[nodiscard]] auto requiredRaw(const ::ad::config::TextConfig &cfg, std::string_view section,
                               std::string_view key) -> Result<std::string_view> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Required localization config key is missing: " +
                                                std::string{section} + "." + std::string{key}});
  }
  return *raw;
}

[[nodiscard]] auto requiredDouble(const ::ad::config::TextConfig &cfg, std::string_view section,
                                  std::string_view key) -> Result<double> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return ::ad::config::parseDoubleValue(*raw);
}

[[nodiscard]] auto requiredInt(const ::ad::config::TextConfig &cfg, std::string_view section,
                               std::string_view key) -> Result<int> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return ::ad::config::parseIntValue(*raw);
}

[[nodiscard]] auto parseEkfConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<EkfLocalizerConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Localization config is required for ekf."});
  }

  const auto &cfg = *configDoc;
  const auto processNoiseTranslation =
      requiredDouble(cfg, "localization.localizer.ekf", "process_noise_translation");
  if (!processNoiseTranslation) {
    return tl::make_unexpected(processNoiseTranslation.error());
  }
  const auto processNoiseRotation =
      requiredDouble(cfg, "localization.localizer.ekf", "process_noise_rotation");
  if (!processNoiseRotation) {
    return tl::make_unexpected(processNoiseRotation.error());
  }
  return EkfLocalizerConfig{.ekf = EkfConfig{.processNoiseTranslation = *processNoiseTranslation,
                                             .processNoiseRotation = *processNoiseRotation}};
}

[[nodiscard]] auto
parseObservationModelType(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::string> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Localization config is required for ekf."});
  }

  const auto raw = requiredRaw(*configDoc, "localization", "observation_model");
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }

  const auto parsed = ::ad::config::parseQuotedString(*raw);
  if (!parsed) {
    return tl::make_unexpected(parsed.error());
  }
  return *parsed;
}

[[nodiscard]] auto
parseSimpleLineAssociationModelConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<SimpleLineAssociationModelConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Localization config is required for simple_line_association."});
  }

  const auto &cfg = *configDoc;
  const auto maxLines =
      requiredInt(cfg, "localization.observation.line_based.map_line_extraction", "max_lines");
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto minSegmentLength = requiredDouble(
      cfg, "localization.observation.line_based.map_line_extraction", "min_segment_length");
  if (!minSegmentLength) {
    return tl::make_unexpected(minSegmentLength.error());
  }
  const auto measurementNoiseRange =
      requiredDouble(cfg, "localization.observation.line_based", "measurement_noise_range");
  if (!measurementNoiseRange) {
    return tl::make_unexpected(measurementNoiseRange.error());
  }
  const auto measurementNoiseAngle =
      requiredDouble(cfg, "localization.observation.line_based", "measurement_noise_angle");
  if (!measurementNoiseAngle) {
    return tl::make_unexpected(measurementNoiseAngle.error());
  }
  const auto maxAssociationDistance = requiredDouble(
      cfg, "localization.observation.models.simple_line_association", "max_association_distance");
  if (!maxAssociationDistance) {
    return tl::make_unexpected(maxAssociationDistance.error());
  }
  const auto segmentMargin = requiredDouble(
      cfg, "localization.observation.models.simple_line_association", "segment_margin");
  if (!segmentMargin) {
    return tl::make_unexpected(segmentMargin.error());
  }
  const auto gateThreshold =
      requiredDouble(cfg, "localization.observation.line_based", "gate_threshold");
  if (!gateThreshold) {
    return tl::make_unexpected(gateThreshold.error());
  }
  const auto minObservations =
      requiredInt(cfg, "localization.observation.line_based", "min_observations");
  if (!minObservations) {
    return tl::make_unexpected(minObservations.error());
  }

  return SimpleLineAssociationModelConfig{
      .mapLineExtraction =
          line_extractor::MapLineExtractionConfig{.maxLines = *maxLines,
                                                  .minSegmentLength = *minSegmentLength},
      .measurementNoiseRange = *measurementNoiseRange,
      .measurementNoiseAngle = *measurementNoiseAngle,
      .maxAssociationDistance = *maxAssociationDistance,
      .segmentMargin = *segmentMargin,
      .gateThreshold = *gateThreshold,
      .minObservations = static_cast<std::size_t>(*minObservations)};
}

struct RansacStage1HybridSettings {
  int maxContinuityGap;
};

[[nodiscard]] auto parseRansacStage1HybridSettings(const ::ad::config::TextConfig &cfg)
    -> Result<RansacStage1HybridSettings> {
  const auto section =
      std::string_view{"localization.observation.models.ransac_line_association.stage1"};
  const auto maxContinuityGap = requiredInt(cfg, section, "max_continuity_gap");
  if (!maxContinuityGap) {
    return tl::make_unexpected(maxContinuityGap.error());
  }

  return RansacStage1HybridSettings{.maxContinuityGap = *maxContinuityGap};
}

[[nodiscard]] auto
parseRansacLineAssociationModelConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<RansacLineAssociationModelConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Localization config is required for ransac_line_association."});
  }

  const auto &cfg = *configDoc;
  const auto maxLines =
      requiredInt(cfg, "localization.observation.line_based.map_line_extraction", "max_lines");
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto mapMinSegmentLength = requiredDouble(
      cfg, "localization.observation.line_based.map_line_extraction", "min_segment_length");
  if (!mapMinSegmentLength) {
    return tl::make_unexpected(mapMinSegmentLength.error());
  }

  const auto measurementNoiseRange =
      requiredDouble(cfg, "localization.observation.line_based", "measurement_noise_range");
  if (!measurementNoiseRange) {
    return tl::make_unexpected(measurementNoiseRange.error());
  }
  const auto measurementNoiseAngle =
      requiredDouble(cfg, "localization.observation.line_based", "measurement_noise_angle");
  if (!measurementNoiseAngle) {
    return tl::make_unexpected(measurementNoiseAngle.error());
  }
  const auto minObservations =
      requiredInt(cfg, "localization.observation.line_based", "min_observations");
  if (!minObservations) {
    return tl::make_unexpected(minObservations.error());
  }

  const auto pointDistanceThreshold =
      requiredDouble(cfg, "localization.observation.models.ransac_line_association.stage1",
                     "point_distance_threshold");
  if (!pointDistanceThreshold) {
    return tl::make_unexpected(pointDistanceThreshold.error());
  }
  const auto minInlierPoints = requiredInt(
      cfg, "localization.observation.models.ransac_line_association.stage1", "min_inlier_points");
  if (!minInlierPoints) {
    return tl::make_unexpected(minInlierPoints.error());
  }
  const auto pointRansacIterations = requiredInt(
      cfg, "localization.observation.models.ransac_line_association.stage1", "max_iterations");
  if (!pointRansacIterations) {
    return tl::make_unexpected(pointRansacIterations.error());
  }
  const auto maxExtractedScanLines =
      requiredInt(cfg, "localization.observation.models.ransac_line_association.stage1",
                  "max_extracted_scan_lines");
  if (!maxExtractedScanLines) {
    return tl::make_unexpected(maxExtractedScanLines.error());
  }
  const auto minExtractedSegmentLength = requiredDouble(
      cfg, "localization.observation.models.ransac_line_association.stage1", "min_segment_length");
  if (!minExtractedSegmentLength) {
    return tl::make_unexpected(minExtractedSegmentLength.error());
  }
  const auto minRemainingPoints =
      requiredInt(cfg, "localization.observation.models.ransac_line_association.stage1",
                  "min_remaining_points");
  if (!minRemainingPoints) {
    return tl::make_unexpected(minRemainingPoints.error());
  }
  const auto hybrid = parseRansacStage1HybridSettings(cfg);
  if (!hybrid) {
    return tl::make_unexpected(hybrid.error());
  }

  const auto translationRansacIterations =
      requiredInt(cfg, "localization.observation.models.ransac_line_association.stage2",
                  "translation_ransac_iterations");
  if (!translationRansacIterations) {
    return tl::make_unexpected(translationRansacIterations.error());
  }
  const auto lineAngleThreshold =
      requiredDouble(cfg, "localization.observation.models.ransac_line_association.stage2",
                     "line_angle_threshold");
  if (!lineAngleThreshold) {
    return tl::make_unexpected(lineAngleThreshold.error());
  }
  const auto lineRhoThreshold = requiredDouble(
      cfg, "localization.observation.models.ransac_line_association.stage2", "line_rho_threshold");
  if (!lineRhoThreshold) {
    return tl::make_unexpected(lineRhoThreshold.error());
  }
  const auto parallelRejectThreshold =
      requiredDouble(cfg, "localization.observation.models.ransac_line_association.stage2",
                     "parallel_reject_threshold");
  if (!parallelRejectThreshold) {
    return tl::make_unexpected(parallelRejectThreshold.error());
  }
  const auto minPoseInliers = requiredInt(
      cfg, "localization.observation.models.ransac_line_association.stage2", "min_pose_inliers");
  if (!minPoseInliers) {
    return tl::make_unexpected(minPoseInliers.error());
  }
  const auto contextGateThreshold =
      requiredDouble(cfg, "localization.observation.models.ransac_line_association.stage4",
                     "context_gate_threshold");
  if (!contextGateThreshold) {
    return tl::make_unexpected(contextGateThreshold.error());
  }

  const auto segmentMargin = requiredDouble(
      cfg, "localization.observation.models.ransac_line_association", "segment_margin");
  if (!segmentMargin) {
    return tl::make_unexpected(segmentMargin.error());
  }
  const auto gateThreshold =
      requiredDouble(cfg, "localization.observation.line_based", "gate_threshold");
  if (!gateThreshold) {
    return tl::make_unexpected(gateThreshold.error());
  }

  const auto useEkfGateRaw =
      requiredRaw(cfg, "localization.observation.models.ransac_line_association", "use_ekf_gate");
  if (!useEkfGateRaw) {
    return tl::make_unexpected(useEkfGateRaw.error());
  }
  const auto useEkfGate = ::ad::config::parseBoolValue(*useEkfGateRaw);
  if (!useEkfGate) {
    return tl::make_unexpected(useEkfGate.error());
  }

  return RansacLineAssociationModelConfig{
      .mapLineExtraction =
          line_extractor::MapLineExtractionConfig{.maxLines = *maxLines,
                                                  .minSegmentLength = *mapMinSegmentLength},
      .measurementNoiseRange = *measurementNoiseRange,
      .measurementNoiseAngle = *measurementNoiseAngle,
      .minObservations = static_cast<std::size_t>(*minObservations),
      .pointDistanceThreshold = *pointDistanceThreshold,
      .minInlierPoints = static_cast<std::size_t>(*minInlierPoints),
      .pointRansacMaxIterations = *pointRansacIterations,
      .maxExtractedScanLines = *maxExtractedScanLines,
      .minExtractedSegmentLength = *minExtractedSegmentLength,
      .minRemainingPoints = static_cast<std::size_t>(*minRemainingPoints),
      .maxContinuityGap = hybrid->maxContinuityGap,
      .translationRansacMaxIterations = *translationRansacIterations,
      .lineAngleThreshold = *lineAngleThreshold,
      .lineRhoThreshold = *lineRhoThreshold,
      .parallelRejectThreshold = *parallelRejectThreshold,
      .minPoseInliers = static_cast<std::size_t>(*minPoseInliers),
      .segmentMargin = *segmentMargin,
      .contextGateThreshold = *contextGateThreshold,
      .useEkfGate = *useEkfGate,
      .gateThreshold = *gateThreshold};
}

} // namespace

auto parseInitialCovarianceFromConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<CovarianceMatrix> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Localization config is required for initial_covariance."});
  }

  CovarianceMatrix covariance = CovarianceMatrix::Zero();
  const auto &cfg = *configDoc;
  const auto readDiagonal = [&](std::string_view key) -> Result<double> {
    return requiredDouble(cfg, "localization.localizer.initial_covariance", key);
  };

  const auto covXx = readDiagonal("xx");
  if (!covXx) {
    return tl::make_unexpected(covXx.error());
  }
  covariance(0, 0) = *covXx;

  const auto covYy = readDiagonal("yy");
  if (!covYy) {
    return tl::make_unexpected(covYy.error());
  }
  covariance(1, 1) = *covYy;

  const auto covTt = readDiagonal("tt");
  if (!covTt) {
    return tl::make_unexpected(covTt.error());
  }
  covariance(2, 2) = *covTt;

  if (covariance(0, 0) <= 0.0 || covariance(1, 1) <= 0.0 || covariance(2, 2) <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "initial_covariance diagonal entries must be positive."});
  }

  return covariance;
}

auto createLocalizerFromConfig(std::string_view algorithm, const types::MapData &map,
                               const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<ILocalizer>> {
  if (algorithm != "ekf") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported localization algorithm: " + std::string{algorithm}});
  }

  const auto configValue = parseEkfConfig(configDoc);
  if (!configValue) {
    return tl::make_unexpected(configValue.error());
  }

  const auto observationModelType = parseObservationModelType(configDoc);
  if (!observationModelType) {
    return tl::make_unexpected(observationModelType.error());
  }

  auto observationModel = std::unique_ptr<IObservationModel>{};
  if (*observationModelType == "simple_line_association") {
    const auto modelConfig = parseSimpleLineAssociationModelConfig(configDoc);
    if (!modelConfig) {
      return tl::make_unexpected(modelConfig.error());
    }

    auto model = SimpleLineAssociationModel::create(map, *modelConfig);
    if (!model) {
      return tl::make_unexpected(model.error());
    }
    observationModel = std::move(*model);
  } else if (*observationModelType == "ransac_line_association") {
    const auto modelConfig = parseRansacLineAssociationModelConfig(configDoc);
    if (!modelConfig) {
      return tl::make_unexpected(modelConfig.error());
    }

    auto model = RansacLineAssociationModel::create(map, *modelConfig);
    if (!model) {
      return tl::make_unexpected(model.error());
    }
    observationModel = std::move(*model);
  } else {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported observation model: " + *observationModelType});
  }

  auto localizer = EkfLocalizer::create(*configValue, std::move(observationModel));
  if (!localizer) {
    return tl::make_unexpected(localizer.error());
  }

  std::unique_ptr<ILocalizer> base = std::move(*localizer);
  return base;
}

} // namespace ad::localization

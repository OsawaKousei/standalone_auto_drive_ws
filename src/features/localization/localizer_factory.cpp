#include "localizer_factory.hpp"

#include "localization_config.hpp"
#include "localizer/ekf_localizer.hpp"
#include "observation_model/ransac_line_association_model.hpp"
#include "observation_model/simple_line_association_model.hpp"

#include <cstddef>

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

[[nodiscard]] auto requiredRawWithFallback(const ::ad::config::TextConfig &cfg,
                                           std::string_view primarySection,
                                           std::string_view fallbackSection, std::string_view key)
    -> Result<std::string_view> {
  const auto primary = cfg.findRaw(primarySection, key);
  if (primary) {
    return *primary;
  }
  const auto fallback = cfg.findRaw(fallbackSection, key);
  if (fallback) {
    return *fallback;
  }
  return tl::make_unexpected(Error{
      .code = ErrorCode::InvalidInput,
      .message = "Required localization config key is missing: " + std::string{primarySection} +
                 "." + std::string{key} + " (legacy fallback: " + std::string{fallbackSection} +
                 ")"});
}

[[nodiscard]] auto requiredDouble(const ::ad::config::TextConfig &cfg, std::string_view section,
                                  std::string_view key) -> Result<double> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return ::ad::config::parseDoubleValue(*raw);
}

[[nodiscard]] auto requiredDoubleWithFallback(const ::ad::config::TextConfig &cfg,
                                              std::string_view primarySection,
                                              std::string_view fallbackSection,
                                              std::string_view key) -> Result<double> {
  const auto raw = requiredRawWithFallback(cfg, primarySection, fallbackSection, key);
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

[[nodiscard]] auto requiredIntWithFallback(const ::ad::config::TextConfig &cfg,
                                           std::string_view primarySection,
                                           std::string_view fallbackSection, std::string_view key)
    -> Result<int> {
  const auto raw = requiredRawWithFallback(cfg, primarySection, fallbackSection, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return ::ad::config::parseIntValue(*raw);
}

[[nodiscard]] auto optionalDouble(const ::ad::config::TextConfig &cfg, std::string_view section,
                                  std::string_view key, double defaultValue) -> Result<double> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return defaultValue;
  }
  return ::ad::config::parseDoubleValue(*raw);
}

[[nodiscard]] auto optionalInt(const ::ad::config::TextConfig &cfg, std::string_view section,
                               std::string_view key, int defaultValue) -> Result<int> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return defaultValue;
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
  const auto processNoiseTranslation = requiredDouble(cfg, "ekf", "process_noise_translation");
  if (!processNoiseTranslation) {
    return tl::make_unexpected(processNoiseTranslation.error());
  }
  const auto processNoiseRotation = requiredDouble(cfg, "ekf", "process_noise_rotation");
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

  const auto raw = configDoc->findRaw("", "observation_model");
  if (!raw) {
    return std::string{"simple_line_association"};
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
  const auto maxLines = requiredIntWithFallback(cfg, "line_extraction", "hough", "max_lines");
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto minSegmentLength =
      requiredDoubleWithFallback(cfg, "line_extraction", "hough", "min_segment_length");
  if (!minSegmentLength) {
    return tl::make_unexpected(minSegmentLength.error());
  }
  const auto measurementNoiseRange = requiredDouble(cfg, "ekf", "measurement_noise_range");
  if (!measurementNoiseRange) {
    return tl::make_unexpected(measurementNoiseRange.error());
  }
  const auto measurementNoiseAngle = requiredDouble(cfg, "ekf", "measurement_noise_angle");
  if (!measurementNoiseAngle) {
    return tl::make_unexpected(measurementNoiseAngle.error());
  }
  const auto maxAssociationDistance =
      requiredDouble(cfg, "association", "max_association_distance");
  if (!maxAssociationDistance) {
    return tl::make_unexpected(maxAssociationDistance.error());
  }
  const auto segmentMargin = requiredDouble(cfg, "association", "segment_margin");
  if (!segmentMargin) {
    return tl::make_unexpected(segmentMargin.error());
  }
  const auto gateThreshold = requiredDouble(cfg, "association", "gate_threshold");
  if (!gateThreshold) {
    return tl::make_unexpected(gateThreshold.error());
  }
  const auto minObservations = requiredInt(cfg, "association", "min_observations");
  if (!minObservations) {
    return tl::make_unexpected(minObservations.error());
  }

  return SimpleLineAssociationModelConfig{
      .mapLineExtraction =
          MapLineExtractionConfig{.maxLines = *maxLines, .minSegmentLength = *minSegmentLength},
      .measurementNoiseRange = *measurementNoiseRange,
      .measurementNoiseAngle = *measurementNoiseAngle,
      .maxAssociationDistance = *maxAssociationDistance,
      .segmentMargin = *segmentMargin,
      .gateThreshold = *gateThreshold,
      .minObservations = static_cast<std::size_t>(*minObservations)};
}

[[nodiscard]] auto
parseRansacLineAssociationModelConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<RansacLineAssociationModelConfig> {
  const auto baseObservation = parseSimpleLineAssociationModelConfig(configDoc);
  if (!baseObservation) {
    return tl::make_unexpected(baseObservation.error());
  }

  const auto &cfg = *configDoc;
  const auto inlierDistance =
      requiredDoubleWithFallback(cfg, "line_extraction", "hough", "inlier_distance");
  if (!inlierDistance) {
    return tl::make_unexpected(inlierDistance.error());
  }
  const auto mergeRho = requiredDoubleWithFallback(cfg, "line_extraction", "hough", "merge_rho");
  if (!mergeRho) {
    return tl::make_unexpected(mergeRho.error());
  }
  const auto mergeTheta =
      requiredDoubleWithFallback(cfg, "line_extraction", "hough", "merge_theta");
  if (!mergeTheta) {
    return tl::make_unexpected(mergeTheta.error());
  }
  const auto maxIterations = optionalInt(cfg, "ransac", "max_iterations", 80);
  if (!maxIterations) {
    return tl::make_unexpected(maxIterations.error());
  }
  const auto minInliers = optionalInt(cfg, "ransac", "min_inliers", 8);
  if (!minInliers) {
    return tl::make_unexpected(minInliers.error());
  }
  const auto minInlierRatio = optionalDouble(cfg, "ransac", "min_inlier_ratio", 0.35);
  if (!minInlierRatio) {
    return tl::make_unexpected(minInlierRatio.error());
  }

  if (*maxIterations <= 0 || *minInliers < 2 || *minInlierRatio <= 0.0 || *minInlierRatio > 1.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "RANSAC configuration is invalid in [ransac] section."});
  }

  return RansacLineAssociationModelConfig{
      .baseObservation = *baseObservation,
      .ransacLineExtraction =
          RansacLineExtractionConfig{.maxLines = baseObservation->mapLineExtraction.maxLines,
                                     .inlierDistance = *inlierDistance,
                                     .minSegmentLength =
                                         baseObservation->mapLineExtraction.minSegmentLength,
                                     .mergeRho = *mergeRho,
                                     .mergeTheta = *mergeTheta},
      .ransac = RansacConfig{.maxIterations = *maxIterations,
                             .minInliers = static_cast<std::size_t>(*minInliers),
                             .minInlierRatio = *minInlierRatio}};
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
    return requiredDouble(cfg, "initial_covariance", key);
  };

  const auto xx = readDiagonal("xx");
  if (!xx) {
    return tl::make_unexpected(xx.error());
  }
  covariance(0, 0) = *xx;

  const auto yy = readDiagonal("yy");
  if (!yy) {
    return tl::make_unexpected(yy.error());
  }
  covariance(1, 1) = *yy;

  const auto tt = readDiagonal("tt");
  if (!tt) {
    return tl::make_unexpected(tt.error());
  }
  covariance(2, 2) = *tt;

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

  std::unique_ptr<IObservationModel> observationModel;
  if (*observationModelType == "hough_line" || *observationModelType == "simple_line_association") {
    const auto modelConfig = parseSimpleLineAssociationModelConfig(configDoc);
    if (!modelConfig) {
      return tl::make_unexpected(modelConfig.error());
    }
    auto model = SimpleLineAssociationModel::create(map, *modelConfig);
    if (!model) {
      return tl::make_unexpected(model.error());
    }
    observationModel = std::move(*model);
  } else if (*observationModelType == "hough_ransac_line" ||
             *observationModelType == "ransac_line_association") {
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

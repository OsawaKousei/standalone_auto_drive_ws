#include "localizer_factory.hpp"

#include "i_observation_model.hpp"
#include "localizer/ekf_localizer.hpp"
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
  const auto maxLines = requiredInt(cfg, "line_extraction", "max_lines");
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto minSegmentLength = requiredDouble(cfg, "line_extraction", "min_segment_length");
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
          line_extractor::MapLineExtractionConfig{.maxLines = *maxLines,
                                                  .minSegmentLength = *minSegmentLength},
      .measurementNoiseRange = *measurementNoiseRange,
      .measurementNoiseAngle = *measurementNoiseAngle,
      .maxAssociationDistance = *maxAssociationDistance,
      .segmentMargin = *segmentMargin,
      .gateThreshold = *gateThreshold,
      .minObservations = static_cast<std::size_t>(*minObservations)};
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

  if (*observationModelType != "simple_line_association") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Unsupported observation model: " + *observationModelType});
  }

  const auto modelConfig = parseSimpleLineAssociationModelConfig(configDoc);
  if (!modelConfig) {
    return tl::make_unexpected(modelConfig.error());
  }

  auto model = SimpleLineAssociationModel::create(map, *modelConfig);
  if (!model) {
    return tl::make_unexpected(model.error());
  }
  std::unique_ptr<IObservationModel> observationModel = std::move(*model);

  auto localizer = EkfLocalizer::create(*configValue, std::move(observationModel));
  if (!localizer) {
    return tl::make_unexpected(localizer.error());
  }

  std::unique_ptr<ILocalizer> base = std::move(*localizer);
  return base;
}

} // namespace ad::localization

#include "localizer_factory.hpp"

#include "ekf_localizer.hpp"
#include "hough_observation_model.hpp"
#include "localization_config.hpp"

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
    return std::string{"hough_line"};
  }

  const auto parsed = ::ad::config::parseQuotedString(*raw);
  if (!parsed) {
    return tl::make_unexpected(parsed.error());
  }
  return *parsed;
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
  if (*observationModelType == "hough_line") {
    auto model = HoughObservationModel::createFromConfig(map, configDoc);
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

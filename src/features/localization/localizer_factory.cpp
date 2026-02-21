#include "localizer_factory.hpp"

#include "ekf_localizer.hpp"
#include "localization_config.hpp"

#include <cstddef>

namespace ad::localization {

namespace {

[[nodiscard]] auto makeDefaultInitialCovariance() -> CovarianceMatrix {
  CovarianceMatrix covariance = CovarianceMatrix::Zero();
  covariance(0, 0) = 0.5;
  covariance(1, 1) = 0.5;
  covariance(2, 2) = 0.2;
  return covariance;
}

[[nodiscard]] auto parseEkfConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<EkfLocalizerConfig> {
  const auto configValue = config::ekfLocalizerDefaultConfig();
  if (!configDoc.has_value()) {
    return configValue;
  }

  const auto &cfg = *configDoc;
  const auto readInt = [&](std::string_view section, std::string_view key,
                           int current) -> Result<int> {
    const auto raw = cfg.findRaw(section, key);
    if (!raw) {
      return current;
    }
    return ::ad::config::parseIntValue(*raw);
  };
  const auto readDouble = [&](std::string_view section, std::string_view key,
                              double current) -> Result<double> {
    const auto raw = cfg.findRaw(section, key);
    if (!raw) {
      return current;
    }
    return ::ad::config::parseDoubleValue(*raw);
  };

  const auto thetaBins = readInt("hough", "theta_bins", configValue.hough.thetaBins);
  if (!thetaBins) {
    return tl::make_unexpected(thetaBins.error());
  }
  const auto rhoBins = readInt("hough", "rho_bins", configValue.hough.rhoBins);
  if (!rhoBins) {
    return tl::make_unexpected(rhoBins.error());
  }
  const auto minVotes = readInt("hough", "min_votes", configValue.hough.minVotes);
  if (!minVotes) {
    return tl::make_unexpected(minVotes.error());
  }
  const auto maxLines = readInt("hough", "max_lines", configValue.hough.maxLines);
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto inlierDistance =
      readDouble("hough", "inlier_distance", configValue.hough.inlierDistance);
  if (!inlierDistance) {
    return tl::make_unexpected(inlierDistance.error());
  }
  const auto minSegmentLength =
      readDouble("hough", "min_segment_length", configValue.hough.minSegmentLength);
  if (!minSegmentLength) {
    return tl::make_unexpected(minSegmentLength.error());
  }
  const auto mergeRho = readDouble("hough", "merge_rho", configValue.hough.mergeRho);
  if (!mergeRho) {
    return tl::make_unexpected(mergeRho.error());
  }
  const auto mergeTheta = readDouble("hough", "merge_theta", configValue.hough.mergeTheta);
  if (!mergeTheta) {
    return tl::make_unexpected(mergeTheta.error());
  }

  const auto processNoiseTranslation =
      readDouble("ekf", "process_noise_translation", configValue.ekf.processNoiseTranslation);
  if (!processNoiseTranslation) {
    return tl::make_unexpected(processNoiseTranslation.error());
  }
  const auto processNoiseRotation =
      readDouble("ekf", "process_noise_rotation", configValue.ekf.processNoiseRotation);
  if (!processNoiseRotation) {
    return tl::make_unexpected(processNoiseRotation.error());
  }
  const auto measurementNoiseRange =
      readDouble("ekf", "measurement_noise_range", configValue.ekf.measurementNoiseRange);
  if (!measurementNoiseRange) {
    return tl::make_unexpected(measurementNoiseRange.error());
  }
  const auto measurementNoiseAngle =
      readDouble("ekf", "measurement_noise_angle", configValue.ekf.measurementNoiseAngle);
  if (!measurementNoiseAngle) {
    return tl::make_unexpected(measurementNoiseAngle.error());
  }

  const auto maxAssociationDistance =
      readDouble("association", "max_association_distance", configValue.maxAssociationDistance);
  if (!maxAssociationDistance) {
    return tl::make_unexpected(maxAssociationDistance.error());
  }
  const auto segmentMargin = readDouble("association", "segment_margin", configValue.segmentMargin);
  if (!segmentMargin) {
    return tl::make_unexpected(segmentMargin.error());
  }
  const auto gateThreshold = readDouble("association", "gate_threshold", configValue.gateThreshold);
  if (!gateThreshold) {
    return tl::make_unexpected(gateThreshold.error());
  }
  const auto minObservations =
      readInt("association", "min_observations", static_cast<int>(configValue.minObservations));
  if (!minObservations) {
    return tl::make_unexpected(minObservations.error());
  }

  return EkfLocalizerConfig{.hough = HoughConfig{.thetaBins = *thetaBins,
                                                 .rhoBins = *rhoBins,
                                                 .minVotes = *minVotes,
                                                 .maxLines = *maxLines,
                                                 .inlierDistance = *inlierDistance,
                                                 .minSegmentLength = *minSegmentLength,
                                                 .mergeRho = *mergeRho,
                                                 .mergeTheta = *mergeTheta},
                            .ekf = EkfConfig{.processNoiseTranslation = *processNoiseTranslation,
                                             .processNoiseRotation = *processNoiseRotation,
                                             .measurementNoiseRange = *measurementNoiseRange,
                                             .measurementNoiseAngle = *measurementNoiseAngle},
                            .maxAssociationDistance = *maxAssociationDistance,
                            .segmentMargin = *segmentMargin,
                            .gateThreshold = *gateThreshold,
                            .minObservations = static_cast<std::size_t>(*minObservations)};
}

} // namespace

auto parseInitialCovarianceFromConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<CovarianceMatrix> {
  auto covariance = makeDefaultInitialCovariance();
  if (!configDoc.has_value()) {
    return covariance;
  }

  const auto &cfg = *configDoc;
  const auto readDiagonal = [&](std::string_view key, int index) -> Result<void> {
    const auto raw = cfg.findRaw("initial_covariance", key);
    if (!raw) {
      return {};
    }
    const auto parsed = ::ad::config::parseDoubleValue(*raw);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    covariance(index, index) = *parsed;
    return {};
  };

  const auto xxStatus = readDiagonal("xx", 0);
  if (!xxStatus) {
    return tl::make_unexpected(xxStatus.error());
  }
  const auto yyStatus = readDiagonal("yy", 1);
  if (!yyStatus) {
    return tl::make_unexpected(yyStatus.error());
  }
  const auto ttStatus = readDiagonal("tt", 2);
  if (!ttStatus) {
    return tl::make_unexpected(ttStatus.error());
  }

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

  auto localizer = EkfLocalizer::create(map, *configValue);
  if (!localizer) {
    return tl::make_unexpected(localizer.error());
  }

  auto derived = std::move(*localizer);
  std::unique_ptr<ILocalizer> base{derived.release()};
  return base;
}

} // namespace ad::localization

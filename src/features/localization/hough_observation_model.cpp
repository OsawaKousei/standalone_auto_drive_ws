#include "hough_observation_model.hpp"

#include "localizer_util.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>
#include <vector>

namespace {

constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;
constexpr double kEpsilon = 1e-9;

using Mat3 = ad::localization::CovarianceMatrix;

struct HoughCandidate {
  double rho;
  double alpha;
  int votes;
};

struct HoughParams {
  double thetaMin;
  double thetaStep;
  double rhoMin;
  double rhoStep;
  int thetaBins;
  int rhoBins;
  std::size_t thetaBinsSize;
  std::size_t rhoBinsSize;
};

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

[[nodiscard]] auto requiredRaw(const ::ad::config::TextConfig &cfg, std::string_view section,
                               std::string_view key) -> ad::Result<std::string_view> {
  const auto raw = cfg.findRaw(section, key);
  if (!raw) {
    return tl::make_unexpected(ad::Error{
        .code = ad::ErrorCode::InvalidInput,
        .message = "Required localization config key is missing: " + std::string{section} + "." +
                   std::string{key}});
  }
  return *raw;
}

[[nodiscard]] auto requiredInt(const ::ad::config::TextConfig &cfg, std::string_view section,
                               std::string_view key) -> ad::Result<int> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return ::ad::config::parseIntValue(*raw);
}

[[nodiscard]] auto requiredDouble(const ::ad::config::TextConfig &cfg, std::string_view section,
                                  std::string_view key) -> ad::Result<double> {
  const auto raw = requiredRaw(cfg, section, key);
  if (!raw) {
    return tl::make_unexpected(raw.error());
  }
  return ::ad::config::parseDoubleValue(*raw);
}

[[nodiscard]] auto
parseHoughObservationConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> ad::Result<ad::localization::HoughObservationModelConfig> {
  if (!configDoc.has_value()) {
    return tl::make_unexpected(
        ad::Error{.code = ad::ErrorCode::InvalidInput,
                  .message = "Localization config is required for hough_line."});
  }

  const auto &cfg = *configDoc;
  const auto thetaBins = requiredInt(cfg, "hough", "theta_bins");
  if (!thetaBins) {
    return tl::make_unexpected(thetaBins.error());
  }
  const auto rhoBins = requiredInt(cfg, "hough", "rho_bins");
  if (!rhoBins) {
    return tl::make_unexpected(rhoBins.error());
  }
  const auto minVotes = requiredInt(cfg, "hough", "min_votes");
  if (!minVotes) {
    return tl::make_unexpected(minVotes.error());
  }
  const auto maxLines = requiredInt(cfg, "hough", "max_lines");
  if (!maxLines) {
    return tl::make_unexpected(maxLines.error());
  }
  const auto inlierDistance = requiredDouble(cfg, "hough", "inlier_distance");
  if (!inlierDistance) {
    return tl::make_unexpected(inlierDistance.error());
  }
  const auto minSegmentLength = requiredDouble(cfg, "hough", "min_segment_length");
  if (!minSegmentLength) {
    return tl::make_unexpected(minSegmentLength.error());
  }
  const auto mergeRho = requiredDouble(cfg, "hough", "merge_rho");
  if (!mergeRho) {
    return tl::make_unexpected(mergeRho.error());
  }
  const auto mergeTheta = requiredDouble(cfg, "hough", "merge_theta");
  if (!mergeTheta) {
    return tl::make_unexpected(mergeTheta.error());
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

  return ad::localization::HoughObservationModelConfig{
      .hough = ad::localization::HoughConfig{.thetaBins = *thetaBins,
                                             .rhoBins = *rhoBins,
                                             .minVotes = *minVotes,
                                             .maxLines = *maxLines,
                                             .inlierDistance = *inlierDistance,
                                             .minSegmentLength = *minSegmentLength,
                                             .mergeRho = *mergeRho,
                                             .mergeTheta = *mergeTheta},
      .measurementNoiseRange = *measurementNoiseRange,
      .measurementNoiseAngle = *measurementNoiseAngle,
      .maxAssociationDistance = *maxAssociationDistance,
      .segmentMargin = *segmentMargin,
      .gateThreshold = *gateThreshold,
      .minObservations = static_cast<std::size_t>(*minObservations)};
}

auto buildHoughParams(const ad::localization::HoughConfig &config, double maxRho) -> HoughParams {
  const auto thetaMin = -0.5 * std::numbers::pi;
  const auto thetaMax = 0.5 * std::numbers::pi;
  const auto thetaStep = (thetaMax - thetaMin) / static_cast<double>(config.thetaBins - 1);
  const auto rhoMin = -maxRho;
  const auto rhoMax = maxRho;
  const auto rhoStep = (rhoMax - rhoMin) / static_cast<double>(config.rhoBins - 1);

  return HoughParams{.thetaMin = thetaMin,
                     .thetaStep = thetaStep,
                     .rhoMin = rhoMin,
                     .rhoStep = rhoStep,
                     .thetaBins = config.thetaBins,
                     .rhoBins = config.rhoBins,
                     .thetaBinsSize = static_cast<std::size_t>(config.thetaBins),
                     .rhoBinsSize = static_cast<std::size_t>(config.rhoBins)};
}

auto buildAccumulator(const std::vector<ad::types::Point> &points, const HoughParams &params)
    -> std::vector<int> {
  auto accumulator = std::vector<int>(params.thetaBinsSize * params.rhoBinsSize, 0);

  for (const auto &point : points) {
    for (int thetaIndex = 0; thetaIndex < params.thetaBins; ++thetaIndex) {
      const auto theta = params.thetaMin + (params.thetaStep * static_cast<double>(thetaIndex));
      const auto rho = (point.x * std::cos(theta)) + (point.y * std::sin(theta));
      const auto rhoIndex = static_cast<int>(std::lround((rho - params.rhoMin) / params.rhoStep));
      if (rhoIndex < 0 || rhoIndex >= params.rhoBins) {
        continue;
      }
      const auto index = (static_cast<std::size_t>(thetaIndex) * params.rhoBinsSize) +
                         static_cast<std::size_t>(rhoIndex);
      ++accumulator[index];
    }
  }

  return accumulator;
}

auto collectCandidates(const std::vector<int> &accumulator, const HoughParams &params,
                       const ad::localization::HoughConfig &config) -> std::vector<HoughCandidate> {
  auto candidates = std::vector<HoughCandidate>{};
  for (int thetaIndex = 0; thetaIndex < params.thetaBins; ++thetaIndex) {
    for (int rhoIndex = 0; rhoIndex < params.rhoBins; ++rhoIndex) {
      const auto index = (static_cast<std::size_t>(thetaIndex) * params.rhoBinsSize) +
                         static_cast<std::size_t>(rhoIndex);
      const auto votes = accumulator[index];
      if (votes < config.minVotes) {
        continue;
      }
      const auto theta = params.thetaMin + (params.thetaStep * static_cast<double>(thetaIndex));
      const auto rho = params.rhoMin + (params.rhoStep * static_cast<double>(rhoIndex));
      candidates.push_back({rho, theta, votes});
    }
  }

  return candidates;
}

auto isTooCloseToExisting(const ad::localization::util::LineModel &normalized,
                          const std::vector<ad::localization::util::MapLine> &lines,
                          const ad::localization::HoughConfig &config) -> bool {
  return std::ranges::any_of(lines, [&](const auto &existing) {
    const auto rhoDiff = std::abs(existing.model.rho - normalized.rho);
    const auto alphaDiff =
        std::abs(ad::localization::util::normalizeAngle(existing.model.alpha - normalized.alpha));
    return rhoDiff <= config.mergeRho && alphaDiff <= config.mergeTheta;
  });
}

auto buildLineFromCandidate(const HoughCandidate &candidate,
                            const std::vector<ad::types::Point> &points,
                            const ad::localization::HoughConfig &config,
                            const ad::localization::util::LineModel &normalized)
    -> std::optional<ad::localization::util::MapLine> {
  const auto normalX = std::cos(candidate.alpha);
  const auto normalY = std::sin(candidate.alpha);
  const auto tangentX = -normalY;
  const auto tangentY = normalX;

  auto minProjection = std::optional<double>{};
  auto maxProjection = std::optional<double>{};
  for (const auto &point : points) {
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - candidate.rho);
    if (distance > config.inlierDistance) {
      continue;
    }

    const auto projection = (tangentX * point.x) + (tangentY * point.y);
    if (!minProjection || projection < *minProjection) {
      minProjection = projection;
    }
    if (!maxProjection || projection > *maxProjection) {
      maxProjection = projection;
    }
  }

  if (!minProjection || !maxProjection) {
    return std::nullopt;
  }

  const auto segmentLength = std::abs(*maxProjection - *minProjection);
  if (segmentLength < config.minSegmentLength) {
    return std::nullopt;
  }

  const auto startPoint =
      ad::types::Point{.x = (tangentX * (*minProjection)) + (normalX * candidate.rho),
                       .y = (tangentY * (*minProjection)) + (normalY * candidate.rho)};
  const auto endPoint =
      ad::types::Point{.x = (tangentX * (*maxProjection)) + (normalX * candidate.rho),
                       .y = (tangentY * (*maxProjection)) + (normalY * candidate.rho)};

  const auto segmentDeltaX = endPoint.x - startPoint.x;
  const auto segmentDeltaY = endPoint.y - startPoint.y;
  const auto segmentDistance = std::hypot(segmentDeltaX, segmentDeltaY);
  if (segmentDistance < kEpsilon) {
    return std::nullopt;
  }

  const auto directionX = segmentDeltaX / segmentDistance;
  const auto directionY = segmentDeltaY / segmentDistance;
  auto minProjValue = (directionX * startPoint.x) + (directionY * startPoint.y);
  auto maxProjValue = (directionX * endPoint.x) + (directionY * endPoint.y);
  if (minProjValue > maxProjValue) {
    std::swap(minProjValue, maxProjValue);
  }

  return ad::localization::util::MapLine{ad::types::LineSegment{startPoint, endPoint},
                                         normalized,
                                         directionX,
                                         directionY,
                                         minProjValue,
                                         maxProjValue};
}

auto extractLinesFromMap(const ad::types::MapData &map, const ad::localization::HoughConfig &config)
    -> ad::Result<std::vector<ad::localization::util::MapLine>> {
  if (!ad::localization::util::mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (config.thetaBins < 2 || config.rhoBins < 2 || config.minVotes <= 0 || config.maxLines <= 0) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::InvalidInput, "Hough configuration is invalid."});
  }

  const auto points = ad::localization::util::collectOccupiedPoints(map);
  if (points.empty()) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::EmptyCollection, "Map contains no occupied cells."});
  }

  const auto maxRho = std::hypot(map.width * map.resolution, map.height * map.resolution);
  if (maxRho <= kEpsilon) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::InvalidInput, "Map resolution too small for Hough transform."});
  }

  const auto params = buildHoughParams(config, maxRho);
  const auto accumulator = buildAccumulator(points, params);
  auto candidates = collectCandidates(accumulator, params, config);

  if (candidates.empty()) {
    return tl::make_unexpected(
        ad::Error{ad::ErrorCode::EmptyCollection, "No Hough candidates met the vote threshold."});
  }

  std::ranges::sort(candidates, [](const auto &left, const auto &right) -> bool {
    return left.votes > right.votes;
  });

  auto lines = std::vector<ad::localization::util::MapLine>{};
  for (const auto &candidate : candidates) {
    if (static_cast<int>(lines.size()) >= config.maxLines) {
      break;
    }

    const auto normalized = ad::localization::util::toLineModel(
        ad::localization::util::LineModel{.rho = candidate.rho, .alpha = candidate.alpha});
    if (isTooCloseToExisting(normalized, lines, config)) {
      continue;
    }

    const auto line = buildLineFromCandidate(candidate, points, config, normalized);
    if (!line) {
      continue;
    }
    lines.push_back(*line);
  }

  if (lines.empty()) {
    return tl::make_unexpected(ad::Error{ad::ErrorCode::EmptyCollection,
                                         "No line segments extracted from Hough candidates."});
  }

  return lines;
}

auto buildBuckets(const ad::types::LidarScan &scan,
                  const std::vector<ad::localization::util::MapLine> &mapLines,
                  const ad::types::Pose &pose,
                  const ad::localization::HoughObservationModelConfig &config)
    -> std::vector<std::vector<ad::types::Point>> {
  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);
  auto buckets = std::vector<std::vector<ad::types::Point>>(mapLines.size());

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }

    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    const auto localX = range * std::cos(angle);
    const auto localY = range * std::sin(angle);

    const auto mapX = pose.x + (cosTheta * localX) - (sinTheta * localY);
    const auto mapY = pose.y + (sinTheta * localX) + (cosTheta * localY);

    std::size_t bestIndex = mapLines.size();
    auto bestDistance = std::optional<double>{};
    for (std::size_t lineIndex = 0; lineIndex < mapLines.size(); ++lineIndex) {
      const auto &line = mapLines[lineIndex];
      const auto projection = (line.directionX * mapX) + (line.directionY * mapY);
      if (projection < (line.minProjection - config.segmentMargin) ||
          projection > (line.maxProjection + config.segmentMargin)) {
        continue;
      }
      const auto lineNormalX = std::cos(line.model.alpha);
      const auto lineNormalY = std::sin(line.model.alpha);
      const auto distance = std::abs((lineNormalX * mapX) + (lineNormalY * mapY) - line.model.rho);
      if (distance > config.maxAssociationDistance) {
        continue;
      }
      if (!bestDistance || distance < *bestDistance) {
        bestDistance = distance;
        bestIndex = lineIndex;
      }
    }

    if (bestIndex >= mapLines.size()) {
      continue;
    }

    buckets[bestIndex].push_back(ad::types::Point{.x = localX, .y = localY});
  }

  return buckets;
}

auto buildObservations(const std::vector<std::vector<ad::types::Point>> &buckets,
                       const std::vector<ad::localization::util::MapLine> &mapLines,
                       const ad::types::Pose &pose,
                       const ad::localization::HoughObservationModelConfig &config,
                       const Mat3 &covariance) -> ObservationSummary {
  ObservationSummary summary{};
  summary.observations.reserve(mapLines.size());

  for (std::size_t lineIndex = 0; lineIndex < mapLines.size(); ++lineIndex) {
    const auto &bucket = buckets[lineIndex];
    if (bucket.size() < 2U) {
      continue;
    }

    const auto fit = ad::localization::util::fitLine(bucket);
    if (!fit) {
      continue;
    }

    auto observation = ad::localization::util::makeExpectedLine(
        mapLines[lineIndex].model, ad::types::Pose{.x = pose.x, .y = pose.y, .theta = pose.theta});
    observation.observed = fit->model;
    const auto pointCount = std::max(1.0, static_cast<double>(fit->pointCount));
    const auto baseRangeVar = config.measurementNoiseRange * config.measurementNoiseRange;
    const auto baseAngleVar = config.measurementNoiseAngle * config.measurementNoiseAngle;
    const auto scale = std::max(1.0, kReferencePoints / pointCount);
    const auto minRangeVar = baseRangeVar * kMinRangeVarianceFactor;
    const auto minAngleVar = baseAngleVar * kMinAngleVarianceFactor;
    observation.rangeVariance = std::max(minRangeVar, (baseRangeVar * scale) + fit->mse);
    observation.angleVariance =
        std::max(minAngleVar, (baseAngleVar * scale) + (fit->mse * kAngleMseScale));
    ++summary.candidates;

    if (!ad::localization::util::gateLineObservation(
            observation, ad::localization::util::ObservationGateConfig{
                             .covariance = covariance, .threshold = config.gateThreshold})) {
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

HoughObservationModel::HoughObservationModel(std::vector<util::MapLine> mapLines,
                                             util::MapSignature signature,
                                             HoughObservationModelConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)) {}

auto HoughObservationModel::create(const types::MapData &map, HoughObservationModelConfig config)
    -> Result<std::unique_ptr<HoughObservationModel>> {
  const auto signature = util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = extractLinesFromMap(map, config.hough);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto observationModel =
      std::make_unique<HoughObservationModel>(std::move(*mapLines), *signature, config);
  return {std::move(observationModel)};
}

auto HoughObservationModel::createFromConfig(
    const types::MapData &map, const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<HoughObservationModel>> {
  const auto modelConfig = parseHoughObservationConfig(configDoc);
  if (!modelConfig) {
    return tl::make_unexpected(modelConfig.error());
  }
  return create(map, *modelConfig);
}

auto HoughObservationModel::buildUpdateInput(const types::LidarScan &scan,
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

  const auto buckets = buildBuckets(scan, mapLines_, predictedPose, config_);
  const auto summary =
      buildObservations(buckets, mapLines_, predictedPose, config_, predictedCovariance);

  if (summary.observations.size() < config_.minObservations) {
    return {std::nullopt};
  }

  const auto score = summary.candidates > 0 ? static_cast<double>(summary.gatePassed) /
                                                  static_cast<double>(summary.candidates)
                                            : 0.0;
  return {buildMeasurementData(summary.observations, score)};
}

} // namespace ad::localization

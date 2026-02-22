#include "simple_line_association_model.hpp"

#include "line_extractor.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;

struct LineFit {
  ad::localization::observation_model::util::LineModel model;
  std::size_t pointCount;
  double mse;
};

struct ObservationSummary {
  std::vector<ad::localization::observation_model::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

auto fitLine(const std::vector<ad::types::Point> &points) -> std::optional<LineFit> {
  if (points.size() < 2U) {
    return std::nullopt;
  }

  double meanX = 0.0;
  double meanY = 0.0;
  for (const auto &point : points) {
    meanX += point.x;
    meanY += point.y;
  }
  const auto count = static_cast<double>(points.size());
  meanX /= count;
  meanY /= count;

  auto momentSums = std::array<double, 3>{0.0, 0.0, 0.0};
  for (const auto &point : points) {
    const auto deltaX = point.x - meanX;
    const auto deltaY = point.y - meanY;
    momentSums[0] += deltaX * deltaX;
    momentSums[1] += deltaX * deltaY;
    momentSums[2] += deltaY * deltaY;
  }

  const auto sxx = momentSums[0];
  const auto sxy = momentSums[1];
  const auto syy = momentSums[2];

  if (sxx + syy < kEpsilon) {
    return std::nullopt;
  }

  const auto direction = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  const auto normal = direction + (0.5 * std::numbers::pi);
  const auto normalX = std::cos(normal);
  const auto normalY = std::sin(normal);
  const auto rho = ((normalX * meanX) + (normalY * meanY));

  auto model = ad::localization::observation_model::util::toLineModel(
      ad::localization::observation_model::util::LineModel{.rho = rho, .alpha = normal});

  const auto lineNormalX = std::cos(model.alpha);
  const auto lineNormalY = std::sin(model.alpha);
  double mse = 0.0;
  for (const auto &point : points) {
    const auto distance = ((lineNormalX * point.x) + (lineNormalY * point.y)) - model.rho;
    mse += distance * distance;
  }
  mse /= count;

  return LineFit{.model = model, .pointCount = points.size(), .mse = mse};
}

auto buildBuckets(const ad::types::LidarScan &scan,
                  const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
                  const ad::types::Pose &pose,
                  const ad::localization::SimpleLineAssociationModelConfig &config)
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

      const auto nx = std::cos(line.model.alpha);
      const auto ny = std::sin(line.model.alpha);
      const auto distance = std::abs((nx * mapX) + (ny * mapY) - line.model.rho);
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

auto buildObservations(
    const std::vector<std::vector<ad::types::Point>> &buckets,
    const std::vector<ad::localization::observation_model::util::MapLine> &mapLines,
    const ad::types::Pose &predictedPose,
    const ad::localization::SimpleLineAssociationModelConfig &config,
    const ad::localization::CovarianceMatrix &predictedCovariance) -> ObservationSummary {
  auto summary = ObservationSummary{};
  summary.observations.reserve(mapLines.size());

  for (std::size_t lineIndex = 0; lineIndex < mapLines.size(); ++lineIndex) {
    const auto &bucket = buckets[lineIndex];
    if (bucket.size() < 2U) {
      continue;
    }

    const auto fit = fitLine(bucket);
    if (!fit) {
      continue;
    }

    auto observation = ad::localization::observation_model::util::makeExpectedLine(
        mapLines[lineIndex].model, predictedPose);
    observation.observed = fit->model;
    ad::localization::observation_model::util::applyObservationNoiseFromMse(
        observation,
        ad::localization::observation_model::util::ObservationNoiseConfig{
            .measurementNoiseRange = config.measurementNoiseRange,
            .measurementNoiseAngle = config.measurementNoiseAngle},
        static_cast<double>(fit->pointCount), fit->mse);

    ++summary.candidates;
    if (!ad::localization::observation_model::util::gateLineObservation(
            observation,
            ad::localization::observation_model::util::ObservationGateConfig{
                .covariance = predictedCovariance, .threshold = config.gateThreshold})) {
      continue;
    }

    ++summary.gatePassed;
    summary.observations.push_back(observation);
  }

  return summary;
}

} // namespace

namespace ad::localization {

SimpleLineAssociationModel::SimpleLineAssociationModel(
    std::vector<observation_model::util::MapLine> mapLines,
    observation_model::util::MapSignature signature, SimpleLineAssociationModelConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)) {}

auto SimpleLineAssociationModel::create(const types::MapData &map,
                                        SimpleLineAssociationModelConfig config)
    -> Result<std::unique_ptr<SimpleLineAssociationModel>> {
  const auto signature = observation_model::util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = line_extractor::extractMapLinesFromMap(map, config.mapLineExtraction);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto observationModel =
      std::make_unique<SimpleLineAssociationModel>(std::move(*mapLines), *signature, config);
  return {std::move(observationModel)};
}

auto SimpleLineAssociationModel::buildUpdateInput(const types::LidarScan &scan,
                                                  const types::MapData &map,
                                                  const types::Pose &predictedPose,
                                                  const CovarianceMatrix &predictedCovariance) const
    -> Result<std::optional<ObservationUpdateInput>> {
  if (!observation_model::util::signatureMatches(mapSignature_, map)) {
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
  return {observation_model::util::buildMeasurementData(summary.observations, score)};
}

} // namespace ad::localization

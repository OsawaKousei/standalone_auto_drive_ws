#include "hough_observation_model.hpp"

#include "localizer_util.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace {

constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;

using Mat3 = ad::localization::CovarianceMatrix;

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

auto buildBuckets(const ad::types::LidarScan &scan,
                  const std::vector<ad::localization::util::MapLine> &mapLines,
                  const ad::types::Pose &pose, const ad::localization::EkfLocalizerConfig &config)
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
                       const ad::localization::EkfLocalizerConfig &config, const Mat3 &covariance)
    -> ObservationSummary {
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
    const auto baseRangeVar = config.ekf.measurementNoiseRange * config.ekf.measurementNoiseRange;
    const auto baseAngleVar = config.ekf.measurementNoiseAngle * config.ekf.measurementNoiseAngle;
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
                                             EkfLocalizerConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)) {}

auto HoughObservationModel::create(const types::MapData &map, EkfLocalizerConfig config)
    -> Result<std::unique_ptr<HoughObservationModel>> {
  const auto signature = util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = util::extractLinesFromMap(map, config.hough);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto observationModel =
      std::make_unique<HoughObservationModel>(std::move(*mapLines), *signature, config);
  return {std::move(observationModel)};
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

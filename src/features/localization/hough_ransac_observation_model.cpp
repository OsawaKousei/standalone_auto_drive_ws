#include "hough_ransac_observation_model.hpp"

#include "hough_line_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>
#include <vector>

namespace {

constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;
constexpr double kEpsilon = 1e-9;

using Mat3 = ad::localization::CovarianceMatrix;

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

struct RansacFitResult {
  ad::localization::util::LineFit fit;
  std::size_t inlierCount;
};

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

auto lineFromTwoPoints(const ad::types::Point &p1, const ad::types::Point &p2)
    -> std::optional<ad::localization::util::LineModel> {
  const auto dx = p2.x - p1.x;
  const auto dy = p2.y - p1.y;
  const auto norm = std::hypot(dx, dy);
  if (norm < kEpsilon) {
    return std::nullopt;
  }

  const auto nx = -dy / norm;
  const auto ny = dx / norm;
  const auto rho = (nx * p1.x) + (ny * p1.y);
  const auto alpha = std::atan2(ny, nx);
  return ad::localization::util::toLineModel(
      ad::localization::util::LineModel{.rho = rho, .alpha = alpha});
}

auto collectInliers(const std::vector<ad::types::Point> &points,
                    const ad::localization::util::LineModel &model, double threshold)
    -> std::vector<ad::types::Point> {
  const auto nx = std::cos(model.alpha);
  const auto ny = std::sin(model.alpha);

  auto inliers = std::vector<ad::types::Point>{};
  inliers.reserve(points.size());
  for (const auto &point : points) {
    const auto distance = std::abs((nx * point.x) + (ny * point.y) - model.rho);
    if (distance <= threshold) {
      inliers.push_back(point);
    }
  }

  return inliers;
}

auto fitLineWithRansac(const std::vector<ad::types::Point> &points,
                       const ad::localization::RansacConfig &config)
    -> std::optional<RansacFitResult> {
  if (points.size() < 2U || config.maxIterations <= 0 || !(config.inlierDistance > 0.0)) {
    return std::nullopt;
  }

  std::mt19937 generator(static_cast<std::mt19937::result_type>(points.size() * 2654435761U));
  std::uniform_int_distribution<std::size_t> distribution(0U, points.size() - 1U);

  auto bestFit = std::optional<RansacFitResult>{};
  for (int iteration = 0; iteration < config.maxIterations; ++iteration) {
    const auto index1 = distribution(generator);
    auto index2 = distribution(generator);
    if (index1 == index2) {
      continue;
    }

    auto model = lineFromTwoPoints(points[index1], points[index2]);
    if (!model) {
      continue;
    }

    auto inliers = collectInliers(points, *model, config.inlierDistance);
    if (inliers.size() < config.minInliers) {
      continue;
    }

    const auto inlierRatio =
        static_cast<double>(inliers.size()) / static_cast<double>(points.size());
    if (inlierRatio < config.minInlierRatio) {
      continue;
    }

    const auto fit = ad::localization::util::fitLine(inliers);
    if (!fit) {
      continue;
    }

    const auto candidate = RansacFitResult{.fit = *fit, .inlierCount = inliers.size()};
    if (!bestFit || candidate.inlierCount > bestFit->inlierCount ||
        (candidate.inlierCount == bestFit->inlierCount && candidate.fit.mse < bestFit->fit.mse)) {
      bestFit = candidate;
    }
  }

  return bestFit;
}

auto buildObservations(const std::vector<std::vector<ad::types::Point>> &buckets,
                       const std::vector<ad::localization::util::MapLine> &mapLines,
                       const ad::types::Pose &pose,
                       const ad::localization::HoughRansacObservationModelConfig &config,
                       const Mat3 &covariance) -> ObservationSummary {
  ObservationSummary summary{};
  summary.observations.reserve(mapLines.size());

  for (std::size_t lineIndex = 0; lineIndex < mapLines.size(); ++lineIndex) {
    const auto &bucket = buckets[lineIndex];
    if (bucket.size() < 2U) {
      continue;
    }

    const auto fitResult = fitLineWithRansac(bucket, config.ransac);
    if (!fitResult) {
      continue;
    }

    auto observation = ad::localization::util::makeExpectedLine(
        mapLines[lineIndex].model, ad::types::Pose{.x = pose.x, .y = pose.y, .theta = pose.theta});
    observation.observed = fitResult->fit.model;
    const auto pointCount = std::max(1.0, static_cast<double>(fitResult->inlierCount));
    const auto baseRangeVar = config.houghObservation.measurementNoiseRange *
                              config.houghObservation.measurementNoiseRange;
    const auto baseAngleVar = config.houghObservation.measurementNoiseAngle *
                              config.houghObservation.measurementNoiseAngle;
    const auto scale = std::max(1.0, kReferencePoints / pointCount);
    const auto minRangeVar = baseRangeVar * kMinRangeVarianceFactor;
    const auto minAngleVar = baseAngleVar * kMinAngleVarianceFactor;
    observation.rangeVariance = std::max(minRangeVar, (baseRangeVar * scale) + fitResult->fit.mse);
    observation.angleVariance =
        std::max(minAngleVar, (baseAngleVar * scale) + (fitResult->fit.mse * kAngleMseScale));
    ++summary.candidates;

    if (!ad::localization::util::gateLineObservation(
            observation,
            ad::localization::util::ObservationGateConfig{
                .covariance = covariance, .threshold = config.houghObservation.gateThreshold})) {
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

HoughRansacObservationModel::HoughRansacObservationModel(std::vector<util::MapLine> mapLines,
                                                         util::MapSignature signature,
                                                         HoughRansacObservationModelConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)) {}

auto HoughRansacObservationModel::create(const types::MapData &map,
                                         HoughRansacObservationModelConfig config)
    -> Result<std::unique_ptr<HoughRansacObservationModel>> {
  const auto signature = util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = hough::extractMapLinesFromMap(map, config.houghObservation.hough);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto observationModel =
      std::make_unique<HoughRansacObservationModel>(std::move(*mapLines), *signature, config);
  return {std::move(observationModel)};
}

auto HoughRansacObservationModel::buildUpdateInput(
    const types::LidarScan &scan, const types::MapData &map, const types::Pose &predictedPose,
    const CovarianceMatrix &predictedCovariance) const
    -> Result<std::optional<ObservationUpdateInput>> {
  if (!util::signatureMatches(mapSignature_, map)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map does not match precomputed line features."});
  }

  if (scan.ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan has no ranges."});
  }

  const auto buckets = buildBuckets(scan, mapLines_, predictedPose, config_.houghObservation);
  const auto summary =
      buildObservations(buckets, mapLines_, predictedPose, config_, predictedCovariance);

  if (summary.observations.size() < config_.houghObservation.minObservations) {
    return {std::nullopt};
  }

  const auto score = summary.candidates > 0 ? static_cast<double>(summary.gatePassed) /
                                                  static_cast<double>(summary.candidates)
                                            : 0.0;
  return {buildMeasurementData(summary.observations, score)};
}

} // namespace ad::localization

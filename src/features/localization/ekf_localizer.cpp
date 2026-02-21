#include "ekf_localizer.hpp"

#include "localizer_util.hpp"

#include <Eigen/Dense>
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

using Mat3 = ad::localization::CovarianceMatrix;

struct ObservationSummary {
  std::vector<ad::localization::util::LineObservation> observations;
  int gatePassed = 0;
  int candidates = 0;
};

struct MeasurementData {
  Eigen::VectorXd residual;
  Eigen::MatrixXd measurementMatrix;
  Eigen::MatrixXd measurementNoise;
  Eigen::Index size = 0;
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

auto buildMeasurementData(const std::vector<ad::localization::util::LineObservation> &observations)
    -> MeasurementData {
  MeasurementData data{};
  const auto measurementCount = observations.size() * 2U;
  data.size = static_cast<Eigen::Index>(measurementCount);
  data.residual = Eigen::VectorXd::Zero(data.size);
  data.measurementMatrix = Eigen::MatrixXd::Zero(data.size, 3);
  data.measurementNoise = Eigen::MatrixXd::Zero(data.size, data.size);

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

EkfLocalizer::EkfLocalizer(std::vector<util::MapLine> mapLines, util::MapSignature signature,
                           EkfLocalizerConfig config)
    : config_(std::move(config)), mapLines_(std::move(mapLines)),
      mapSignature_(std::move(signature)), state_{.x = 0.0, .y = 0.0, .theta = 0.0},
      covariance_{CovarianceMatrix::Zero()} {}

auto EkfLocalizer::create(const types::MapData &map, EkfLocalizerConfig config)
    -> Result<std::unique_ptr<EkfLocalizer>> {
  const auto signature = util::mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = util::extractLinesFromMap(map, config.hough);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  auto localizer = std::make_unique<EkfLocalizer>(std::move(*mapLines), *signature, config);
  return {std::move(localizer)};
}

auto EkfLocalizer::reset(const types::Pose &initialPose, const CovarianceMatrix &initialCovariance)
    -> Status {
  state_ = State{.x = initialPose.x, .y = initialPose.y, .theta = initialPose.theta};
  covariance_ = initialCovariance;
  score_ = 0.0;
  hasState_ = true;
  return {};
}

auto EkfLocalizer::predict(const types::Twist &control, double deltaT) -> Status {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  if (deltaT <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Delta time must be positive."});
  }

  const auto cosTheta = std::cos(state_.theta);
  const auto sinTheta = std::sin(state_.theta);
  const auto deltaX = control.v * cosTheta * deltaT;
  const auto deltaY = control.v * sinTheta * deltaT;
  const auto deltaTheta = control.w * deltaT;

  state_ = State{.x = state_.x + deltaX,
                 .y = state_.y + deltaY,
                 .theta = util::normalizeAngle(state_.theta + deltaTheta)};

  const auto f02 = -control.v * sinTheta * deltaT;
  const auto f12 = control.v * cosTheta * deltaT;

  const Mat3 stateTransition = (Mat3() << 1.0, 0.0, f02, 0.0, 1.0, f12, 0.0, 0.0, 1.0).finished();

  Mat3 newCovariance = Mat3::Zero();
  newCovariance = stateTransition * covariance_ * stateTransition.transpose();
  const auto qPos = config_.ekf.processNoiseTranslation * deltaT;
  const auto qRot = config_.ekf.processNoiseRotation * deltaT;
  newCovariance(0, 0) += qPos;
  newCovariance(1, 1) += qPos;
  newCovariance(2, 2) += qRot;
  covariance_ = newCovariance;
  return {};
}

auto EkfLocalizer::update(const types::LidarScan &scan, const types::MapData &map) -> Status {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  if (!util::signatureMatches(mapSignature_, map)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map does not match precomputed line features."});
  }

  if (scan.ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan has no ranges."});
  }
  const auto pose = types::Pose{.x = state_.x, .y = state_.y, .theta = state_.theta};
  const auto buckets = buildBuckets(scan, mapLines_, pose, config_);
  const auto summary = buildObservations(buckets, mapLines_, pose, config_, covariance_);

  if (summary.observations.size() < config_.minObservations) {
    score_ = 0.0;
    return {};
  }
  const auto measurementData = buildMeasurementData(summary.observations);

  Eigen::MatrixXd innovationCovariance = measurementData.measurementMatrix * covariance_ *
                                         measurementData.measurementMatrix.transpose();
  innovationCovariance += measurementData.measurementNoise;
  const auto innovationDecomp = innovationCovariance.ldlt();
  if (innovationDecomp.info() != Eigen::Success) {
    score_ = 0.0;
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "EKF update failed due to singular S matrix."});
  }

  const Eigen::MatrixXd innovationInv =
      innovationDecomp.solve(Eigen::MatrixXd::Identity(measurementData.size, measurementData.size));
  Eigen::MatrixXd kalmanGain =
      covariance_ * measurementData.measurementMatrix.transpose() * innovationInv;
  const Eigen::Vector3d delta = kalmanGain * measurementData.residual;

  state_ = State{.x = state_.x + delta(0),
                 .y = state_.y + delta(1),
                 .theta = util::normalizeAngle(state_.theta + delta(2))};

  const Mat3 kalmanProjection = (kalmanGain * measurementData.measurementMatrix).eval();
  Mat3 newCovariance = Mat3::Identity();
  newCovariance -= kalmanProjection;
  newCovariance *= covariance_;
  covariance_ = newCovariance;

  score_ = summary.candidates > 0
               ? static_cast<double>(summary.gatePassed) / static_cast<double>(summary.candidates)
               : 0.0;
  return {};
}

auto EkfLocalizer::estimate() const -> Result<LocalizerEstimate> {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  return LocalizerEstimate{.pose = types::Pose{.x = state_.x, .y = state_.y, .theta = state_.theta},
                           .covariance = covariance_,
                           .score = score_};
}

} // namespace ad::localization

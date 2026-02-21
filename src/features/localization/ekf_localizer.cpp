#include "ekf_localizer.hpp"

#include "localizer_util.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace ad::localization {

using Mat3 = ad::localization::CovarianceMatrix;
EkfLocalizer::EkfLocalizer(EkfLocalizerConfig config,
                           std::unique_ptr<IObservationModel> observationModel)
    : config_(std::move(config)), observationModel_(std::move(observationModel)),
      state_{.x = 0.0, .y = 0.0, .theta = 0.0}, covariance_{CovarianceMatrix::Zero()} {}

auto EkfLocalizer::create(EkfLocalizerConfig config,
                          std::unique_ptr<IObservationModel> observationModel)
    -> Result<std::unique_ptr<EkfLocalizer>> {
  if (!observationModel) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Observation model is not initialized."});
  }

  auto localizer = std::make_unique<EkfLocalizer>(config, std::move(observationModel));
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

auto EkfLocalizer::predictOdometry(const types::OdometryDelta &delta) -> Status {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  if (!std::isfinite(delta.deltaForward) || !std::isfinite(delta.deltaLateral) ||
      !std::isfinite(delta.deltaTheta)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Odometry delta values must be finite."});
  }

  const auto cosTheta = std::cos(state_.theta);
  const auto sinTheta = std::sin(state_.theta);
  const auto deltaX = (delta.deltaForward * cosTheta) - (delta.deltaLateral * sinTheta);
  const auto deltaY = (delta.deltaForward * sinTheta) + (delta.deltaLateral * cosTheta);
  const auto deltaTheta = delta.deltaTheta;

  state_ = State{.x = state_.x + deltaX,
                 .y = state_.y + deltaY,
                 .theta = util::normalizeAngle(state_.theta + deltaTheta)};

  const auto f02 = (-delta.deltaForward * sinTheta) - (delta.deltaLateral * cosTheta);
  const auto f12 = (delta.deltaForward * cosTheta) - (delta.deltaLateral * sinTheta);

  const Mat3 stateTransition = (Mat3() << 1.0, 0.0, f02, 0.0, 1.0, f12, 0.0, 0.0, 1.0).finished();

  Mat3 newCovariance = Mat3::Zero();
  newCovariance = stateTransition * covariance_ * stateTransition.transpose();
  const auto translationTravel = std::hypot(delta.deltaForward, delta.deltaLateral);
  const auto qPos = config_.ekf.processNoiseTranslation * std::max(translationTravel, 1.0e-6);
  const auto qRot = config_.ekf.processNoiseRotation * std::max(std::abs(deltaTheta), 1.0e-6);
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

  if (!observationModel_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Observation model is not initialized."});
  }

  const auto pose = types::Pose{.x = state_.x, .y = state_.y, .theta = state_.theta};
  const auto measurement = observationModel_->buildUpdateInput(scan, map, pose, covariance_);
  if (!measurement) {
    return tl::make_unexpected(measurement.error());
  }

  if (!measurement->has_value()) {
    score_ = 0.0;
    return {};
  }
  const auto &measurementData = **measurement;
  const auto measurementSize = measurementData.residual.size();

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
      innovationDecomp.solve(Eigen::MatrixXd::Identity(measurementSize, measurementSize));
  const Eigen::MatrixXd kalmanGain =
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

  score_ = measurementData.score;
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

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

using Mat3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

} // namespace

namespace ad::localization {

auto EkfLocalizer::defaultConfig() -> EkfLocalizerConfig {
  return EkfLocalizerConfig{.hough = HoughConfig{.thetaBins = 180,
                                                 .rhoBins = 200,
                                                 .minVotes = 25,
                                                 .maxLines = 40,
                                                 .inlierDistance = 0.12,
                                                 .minSegmentLength = 0.8,
                                                 .mergeRho = 0.2,
                                                 .mergeTheta = 0.08},
                            .ekf = EkfConfig{.processNoiseTranslation = 0.05,
                                             .processNoiseRotation = 0.03,
                                             .measurementNoiseRange = 0.12,
                                             .measurementNoiseAngle = 0.12},
                            .maxAssociationDistance = 0.3,
                            .segmentMargin = 0.3,
                            .gateThreshold = 6.0,
                            .minObservations = 3U};
}

EkfLocalizer::EkfLocalizer(std::vector<util::MapLine> mapLines, util::MapSignature signature,
                           EkfLocalizerConfig config)
    : config_(config), mapLines_(std::move(mapLines)), mapSignature_(signature),
      state_{0.0, 0.0, 0.0}, covariance_{}, score_(0.0), hasState_(false) {
  covariance_.fill(0.0);
}

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

  auto localizer =
      std::unique_ptr<EkfLocalizer>(new EkfLocalizer(std::move(*mapLines), *signature, config));
  return Result<std::unique_ptr<EkfLocalizer>>(std::move(localizer));
}

auto EkfLocalizer::reset(const types::Pose &initialPose,
                         const std::array<double, 9> &initialCovariance) -> Status {
  state_ = State{initialPose.x, initialPose.y, initialPose.theta};
  covariance_ = initialCovariance;
  score_ = 0.0;
  hasState_ = true;
  return {};
}

auto EkfLocalizer::predict(const types::Twist &control, double dt) -> Status {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  if (dt <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Delta time must be positive."});
  }

  const auto cosTheta = std::cos(state_.theta);
  const auto sinTheta = std::sin(state_.theta);
  const auto deltaX = control.v * cosTheta * dt;
  const auto deltaY = control.v * sinTheta * dt;
  const auto deltaTheta = control.w * dt;

  state_ =
      State{state_.x + deltaX, state_.y + deltaY, util::normalizeAngle(state_.theta + deltaTheta)};

  const auto f02 = -control.v * sinTheta * dt;
  const auto f12 = control.v * cosTheta * dt;

  Mat3 f;
  f << 1.0, 0.0, f02, 0.0, 1.0, f12, 0.0, 0.0, 1.0;

  const Eigen::Map<const Mat3> p(covariance_.data());
  Mat3 pNew = (f * p * f.transpose());
  const auto qPos = config_.ekf.processNoiseTranslation * dt;
  const auto qRot = config_.ekf.processNoiseRotation * dt;
  pNew(0, 0) += qPos;
  pNew(1, 1) += qPos;
  pNew(2, 2) += qRot;

  Eigen::Map<Mat3>(covariance_.data()) = pNew;
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

  const auto cosTheta = std::cos(state_.theta);
  const auto sinTheta = std::sin(state_.theta);

  auto buckets = std::vector<std::vector<types::Point>>(mapLines_.size());
  int associationAttempts = 0;
  for (const auto index : std::views::iota(std::size_t{0}, scan.ranges.size())) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }

    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    const auto px = range * std::cos(angle);
    const auto py = range * std::sin(angle);

    const auto mapX = state_.x + (cosTheta * px) - (sinTheta * py);
    const auto mapY = state_.y + (sinTheta * px) + (cosTheta * py);

    std::size_t bestIndex = mapLines_.size();
    auto bestDistance = std::optional<double>{};
    for (std::size_t lineIndex = 0; lineIndex < mapLines_.size(); ++lineIndex) {
      const auto &line = mapLines_[lineIndex];
      const auto projection = (line.directionX * mapX) + (line.directionY * mapY);
      if (projection < (line.minProjection - config_.segmentMargin) ||
          projection > (line.maxProjection + config_.segmentMargin)) {
        continue;
      }
      const auto nx = std::cos(line.model.alpha);
      const auto ny = std::sin(line.model.alpha);
      const auto distance = std::abs((nx * mapX) + (ny * mapY) - line.model.rho);
      if (distance > config_.maxAssociationDistance) {
        continue;
      }
      if (!bestDistance || distance < *bestDistance) {
        bestDistance = distance;
        bestIndex = lineIndex;
      }
    }

    ++associationAttempts;
    if (bestIndex >= mapLines_.size()) {
      continue;
    }

    buckets[bestIndex].push_back(types::Point{.x = px, .y = py});
  }

  auto observations = std::vector<util::LineObservation>{};
  observations.reserve(mapLines_.size());
  int gatePassed = 0;
  int candidates = 0;
  for (std::size_t lineIndex = 0; lineIndex < mapLines_.size(); ++lineIndex) {
    const auto &bucket = buckets[lineIndex];
    if (bucket.size() < 2U) {
      continue;
    }

    const auto fit = util::fitLine(bucket);
    if (!fit) {
      continue;
    }

    auto observation =
        util::makeExpectedLine(mapLines_[lineIndex].model, state_.x, state_.y, state_.theta);
    observation.observed = fit->model;
    const auto pointCount = std::max(1.0, static_cast<double>(fit->pointCount));
    const auto baseRangeVar = config_.ekf.measurementNoiseRange * config_.ekf.measurementNoiseRange;
    const auto baseAngleVar = config_.ekf.measurementNoiseAngle * config_.ekf.measurementNoiseAngle;
    const auto scale = std::max(1.0, kReferencePoints / pointCount);
    const auto minRangeVar = baseRangeVar * kMinRangeVarianceFactor;
    const auto minAngleVar = baseAngleVar * kMinAngleVarianceFactor;
    observation.rangeVariance = std::max(minRangeVar, (baseRangeVar * scale) + fit->mse);
    observation.angleVariance =
        std::max(minAngleVar, (baseAngleVar * scale) + (fit->mse * kAngleMseScale));
    ++candidates;

    if (!util::gateLineObservation(observation, covariance_, config_.gateThreshold)) {
      continue;
    }

    ++gatePassed;
    observations.push_back(observation);
  }

  if (observations.size() < config_.minObservations) {
    score_ = 0.0;
    return {};
  }

  const auto measurementCount = observations.size() * 2U;
  const auto measurementSize = static_cast<Eigen::Index>(measurementCount);
  Eigen::VectorXd residual = Eigen::VectorXd::Zero(measurementSize);
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(measurementSize, 3);
  Eigen::MatrixXd r = Eigen::MatrixXd::Zero(measurementSize, measurementSize);

  for (std::size_t index = 0; index < observations.size(); ++index) {
    const auto &obs = observations[index];
    const auto row = static_cast<Eigen::Index>(index * 2U);

    const auto residualRho = obs.observed.rho - obs.expected.rho;
    const auto residualAlpha = util::normalizeAngle(obs.observed.alpha - obs.expected.alpha);
    residual(row) = residualRho;
    residual(row + 1) = residualAlpha;

    const auto hRhoX = -obs.rhoSign * obs.nx;
    const auto hRhoY = -obs.rhoSign * obs.ny;
    const auto hAlphaTheta = -1.0;

    h(row, 0) = hRhoX;
    h(row, 1) = hRhoY;
    h(row, 2) = 0.0;

    h(row + 1, 0) = 0.0;
    h(row + 1, 1) = 0.0;
    h(row + 1, 2) = hAlphaTheta;

    r(row, row) = obs.rangeVariance;
    r(row + 1, row + 1) = obs.angleVariance;
  }

  const Eigen::Map<const Mat3> p(covariance_.data());
  const Eigen::MatrixXd s = (h * p * h.transpose()) + r;
  const auto sDecomp = s.ldlt();
  if (sDecomp.info() != Eigen::Success) {
    score_ = 0.0;
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "EKF update failed due to singular S matrix."});
  }

  const Eigen::MatrixXd sInv =
      sDecomp.solve(Eigen::MatrixXd::Identity(measurementSize, measurementSize));
  const Eigen::MatrixXd k = (p * h.transpose()) * sInv;
  const Eigen::Vector3d delta = k * residual;

  state_ = State{state_.x + delta(0), state_.y + delta(1),
                 util::normalizeAngle(state_.theta + delta(2))};

  const Mat3 kh = (k * h).eval();
  const Mat3 pNew = (Mat3::Identity() - kh) * p;
  Eigen::Map<Mat3>(covariance_.data()) = pNew;

  score_ = candidates > 0 ? static_cast<double>(gatePassed) / static_cast<double>(candidates) : 0.0;
  (void)associationAttempts;
  return {};
}

auto EkfLocalizer::estimate() const -> Result<LocalizerEstimate> {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  return LocalizerEstimate{.pose = types::Pose{state_.x, state_.y, state_.theta},
                           .covariance = covariance_,
                           .score = score_};
}

} // namespace ad::localization

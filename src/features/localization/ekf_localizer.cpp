#include "ekf_localizer.hpp"

#include "localizer_util.hpp"
#include "shared/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>
#include <vector>

namespace {

constexpr double kGateEpsilon = 1e-12;
constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;

[[nodiscard]] auto multiply(const std::vector<double> &left, std::size_t leftRows,
                            std::size_t leftCols, const std::vector<double> &right,
                            std::size_t rightRows, std::size_t rightCols) -> std::vector<double> {
  (void)rightRows;
  auto result = std::vector<double>(leftRows * rightCols, 0.0);
  for (std::size_t row = 0; row < leftRows; ++row) {
    for (std::size_t col = 0; col < rightCols; ++col) {
      double sum = 0.0;
      for (std::size_t k = 0; k < leftCols; ++k) {
        sum += left[(row * leftCols) + k] * right[(k * rightCols) + col];
      }
      result[(row * rightCols) + col] = sum;
    }
  }
  return result;
}

[[nodiscard]] auto transpose(const std::vector<double> &matrix, std::size_t rows, std::size_t cols)
    -> std::vector<double> {
  auto result = std::vector<double>(rows * cols, 0.0);
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t col = 0; col < cols; ++col) {
      result[(col * rows) + row] = matrix[(row * cols) + col];
    }
  }
  return result;
}

[[nodiscard]] auto invertSquare(const std::vector<double> &matrix, std::size_t size)
    -> std::optional<std::vector<double>> {
  auto augmented = std::vector<double>(size * size * 2U, 0.0);
  for (std::size_t row = 0; row < size; ++row) {
    for (std::size_t col = 0; col < size; ++col) {
      augmented[(row * size * 2U) + col] = matrix[(row * size) + col];
    }
    augmented[(row * size * 2U) + (size + row)] = 1.0;
  }

  for (std::size_t pivot = 0; pivot < size; ++pivot) {
    auto pivotValue = augmented[(pivot * size * 2U) + pivot];
    if (std::abs(pivotValue) < kGateEpsilon) {
      return std::nullopt;
    }
    const auto invPivot = 1.0 / pivotValue;
    for (std::size_t col = 0; col < size * 2U; ++col) {
      augmented[(pivot * size * 2U) + col] *= invPivot;
    }

    for (std::size_t row = 0; row < size; ++row) {
      if (row == pivot) {
        continue;
      }
      const auto factor = augmented[(row * size * 2U) + pivot];
      if (std::abs(factor) < kGateEpsilon) {
        continue;
      }
      for (std::size_t col = 0; col < size * 2U; ++col) {
        augmented[(row * size * 2U) + col] -= factor * augmented[(pivot * size * 2U) + col];
      }
    }
  }

  auto inverse = std::vector<double>(size * size, 0.0);
  for (std::size_t row = 0; row < size; ++row) {
    for (std::size_t col = 0; col < size; ++col) {
      inverse[(row * size) + col] = augmented[(row * size * 2U) + (size + col)];
    }
  }
  return inverse;
}

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

  const auto f = std::array<double, 9>{1.0, 0.0, f02, 0.0, 1.0, f12, 0.0, 0.0, 1.0};
  const auto fp = math::multiply(f, covariance_);
  const auto fpt = math::multiply(fp, math::transpose(f));

  auto pNew = fpt;
  const auto qPos = config_.ekf.processNoiseTranslation * dt;
  const auto qRot = config_.ekf.processNoiseRotation * dt;
  pNew[0] += qPos;
  pNew[4] += qPos;
  pNew[8] += qRot;

  covariance_ = pNew;
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
  auto residual = std::vector<double>(measurementCount, 0.0);
  auto h = std::vector<double>(measurementCount * 3U, 0.0);
  auto r = std::vector<double>(measurementCount * measurementCount, 0.0);

  for (std::size_t index = 0; index < observations.size(); ++index) {
    const auto &obs = observations[index];
    const auto row = index * 2U;

    const auto residualRho = obs.observed.rho - obs.expected.rho;
    const auto residualAlpha = util::normalizeAngle(obs.observed.alpha - obs.expected.alpha);
    residual[row] = residualRho;
    residual[row + 1U] = residualAlpha;

    const auto hRhoX = -obs.rhoSign * obs.nx;
    const auto hRhoY = -obs.rhoSign * obs.ny;
    const auto hAlphaTheta = -1.0;

    h[(row * 3U) + 0U] = hRhoX;
    h[(row * 3U) + 1U] = hRhoY;
    h[(row * 3U) + 2U] = 0.0;

    h[((row + 1U) * 3U) + 0U] = 0.0;
    h[((row + 1U) * 3U) + 1U] = 0.0;
    h[((row + 1U) * 3U) + 2U] = hAlphaTheta;

    r[(row * measurementCount) + row] = obs.rangeVariance;
    r[((row + 1U) * measurementCount) + (row + 1U)] = obs.angleVariance;
  }

  auto p = std::vector<double>(9U, 0.0);
  for (std::size_t index = 0; index < 9U; ++index) {
    p[index] = covariance_[index];
  }

  const auto hT = transpose(h, measurementCount, 3U);
  const auto hp = multiply(h, measurementCount, 3U, p, 3U, 3U);
  auto s = multiply(hp, measurementCount, 3U, hT, 3U, measurementCount);
  for (std::size_t row = 0; row < measurementCount; ++row) {
    for (std::size_t col = 0; col < measurementCount; ++col) {
      s[(row * measurementCount) + col] += r[(row * measurementCount) + col];
    }
  }

  const auto sInv = invertSquare(s, measurementCount);
  if (!sInv) {
    score_ = 0.0;
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "EKF update failed due to singular S matrix."});
  }

  const auto pHt = multiply(p, 3U, 3U, hT, 3U, measurementCount);
  const auto k = multiply(pHt, 3U, measurementCount, *sInv, measurementCount, measurementCount);

  auto delta = std::vector<double>(3U, 0.0);
  for (std::size_t row = 0; row < 3U; ++row) {
    double sum = 0.0;
    for (std::size_t col = 0; col < measurementCount; ++col) {
      sum += k[(row * measurementCount) + col] * residual[col];
    }
    delta[row] = sum;
  }

  state_ = State{state_.x + delta[0], state_.y + delta[1],
                 util::normalizeAngle(state_.theta + delta[2])};

  const auto kh = multiply(k, 3U, measurementCount, h, measurementCount, 3U);
  auto iMinusKh = std::vector<double>(9U, 0.0);
  iMinusKh[0] = 1.0 - kh[0];
  iMinusKh[1] = -kh[1];
  iMinusKh[2] = -kh[2];
  iMinusKh[3] = -kh[3];
  iMinusKh[4] = 1.0 - kh[4];
  iMinusKh[5] = -kh[5];
  iMinusKh[6] = -kh[6];
  iMinusKh[7] = -kh[7];
  iMinusKh[8] = 1.0 - kh[8];

  const auto pNew = multiply(iMinusKh, 3U, 3U, p, 3U, 3U);
  for (std::size_t index = 0; index < 9U; ++index) {
    covariance_[index] = pNew[index];
  }

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

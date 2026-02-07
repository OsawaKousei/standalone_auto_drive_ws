#include "pure_ekf_localizer.hpp"

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

constexpr double kEpsilon = 1e-9;

struct HoughCandidate {
  double rho;
  double alpha;
  int votes;
};

[[nodiscard]] auto toLineModel(double rho, double alpha) -> ad::localization::LineModel {
  auto normalizedRho = rho;
  auto normalizedAlpha = ad::localization::util::normalizeAngle(alpha);
  if (normalizedRho < 0.0) {
    normalizedRho = -normalizedRho;
    normalizedAlpha = ad::localization::util::normalizeAngle(normalizedAlpha + std::numbers::pi);
  }
  return ad::localization::LineModel{normalizedRho, normalizedAlpha};
}

} // namespace

namespace ad::localization {

auto PureEkfLocalizer::defaultConfig() -> PureEkfLocalizerConfig {
  return PureEkfLocalizerConfig{.hough = HoughConfig{.thetaBins = 180,
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
                                                 .measurementNoiseAngle = 0.08},
                                .maxAssociationDistance = 0.3,
                                .gateThreshold = 6.0};
}

PureEkfLocalizer::PureEkfLocalizer(std::vector<MapLine> mapLines, MapSignature signature,
                                   PureEkfLocalizerConfig config)
    : config_(config), mapLines_(std::move(mapLines)), mapSignature_(signature),
      state_{0.0, 0.0, 0.0}, covariance_{}, score_(0.0), hasState_(false) {
  covariance_.fill(0.0);
}

auto PureEkfLocalizer::create(const types::MapData &map, PureEkfLocalizerConfig config)
    -> Result<PureEkfLocalizer> {
  const auto signature = mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = extractLinesFromMap(map, config.hough);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  return PureEkfLocalizer{std::move(*mapLines), *signature, config};
}

auto PureEkfLocalizer::mapSignatureFromMap(const types::MapData &map) -> Result<MapSignature> {
  if (!util::mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (map.width <= 0 || map.height <= 0 || map.resolution <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Map dimensions must be positive."});
  }

  return MapSignature{.width = map.width,
                      .height = map.height,
                      .resolution = map.resolution,
                      .gridSize = map.grid.size()};
}

auto PureEkfLocalizer::signatureMatches(const MapSignature &signature, const types::MapData &map)
    -> bool {
  return signature.width == map.width && signature.height == map.height &&
         signature.resolution == map.resolution && signature.gridSize == map.grid.size();
}

auto PureEkfLocalizer::extractLinesFromMap(const types::MapData &map, const HoughConfig &config)
    -> Result<std::vector<MapLine>> {
  if (!util::mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (config.thetaBins < 2 || config.rhoBins < 2 || config.minVotes <= 0 || config.maxLines <= 0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Hough configuration is invalid."});
  }

  const auto points = util::collectOccupiedPoints(map);
  if (points.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Map contains no occupied cells."});
  }

  const auto maxRho = std::hypot(map.width * map.resolution, map.height * map.resolution);
  if (maxRho <= kEpsilon) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map resolution too small for Hough transform."});
  }

  const auto thetaMin = -0.5 * std::numbers::pi;
  const auto thetaMax = 0.5 * std::numbers::pi;
  const auto thetaStep = (thetaMax - thetaMin) / static_cast<double>(config.thetaBins - 1);
  const auto rhoMin = -maxRho;
  const auto rhoMax = maxRho;
  const auto rhoStep = (rhoMax - rhoMin) / static_cast<double>(config.rhoBins - 1);

  auto accumulator =
      std::vector<int>(static_cast<std::size_t>(config.thetaBins * config.rhoBins), 0);

  for (const auto &point : points) {
    for (int thetaIndex = 0; thetaIndex < config.thetaBins; ++thetaIndex) {
      const auto theta = thetaMin + (thetaStep * static_cast<double>(thetaIndex));
      const auto rho = (point.x * std::cos(theta)) + (point.y * std::sin(theta));
      const auto rhoIndex = static_cast<int>(std::lround((rho - rhoMin) / rhoStep));
      if (rhoIndex < 0 || rhoIndex >= config.rhoBins) {
        continue;
      }
      const auto index = static_cast<std::size_t>((thetaIndex * config.rhoBins) + rhoIndex);
      ++accumulator[index];
    }
  }

  auto candidates = std::vector<HoughCandidate>{};
  for (int thetaIndex = 0; thetaIndex < config.thetaBins; ++thetaIndex) {
    for (int rhoIndex = 0; rhoIndex < config.rhoBins; ++rhoIndex) {
      const auto index = static_cast<std::size_t>((thetaIndex * config.rhoBins) + rhoIndex);
      const auto votes = accumulator[index];
      if (votes < config.minVotes) {
        continue;
      }
      const auto theta = thetaMin + (thetaStep * static_cast<double>(thetaIndex));
      const auto rho = rhoMin + (rhoStep * static_cast<double>(rhoIndex));
      candidates.push_back({rho, theta, votes});
    }
  }

  if (candidates.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No Hough candidates met the vote threshold."});
  }

  std::ranges::sort(candidates,
                    [](const auto &left, const auto &right) { return left.votes > right.votes; });

  auto lines = std::vector<MapLine>{};
  for (const auto &candidate : candidates) {
    if (static_cast<int>(lines.size()) >= config.maxLines) {
      break;
    }

    const auto normalized = toLineModel(candidate.rho, candidate.alpha);
    bool tooClose = false;
    for (const auto &existing : lines) {
      const auto rhoDiff = std::abs(existing.model.rho - normalized.rho);
      const auto alphaDiff =
          std::abs(util::normalizeAngle(existing.model.alpha - normalized.alpha));
      if (rhoDiff <= config.mergeRho && alphaDiff <= config.mergeTheta) {
        tooClose = true;
        break;
      }
    }
    if (tooClose) {
      continue;
    }

    const auto nx = std::cos(candidate.alpha);
    const auto ny = std::sin(candidate.alpha);
    const auto dx = -ny;
    const auto dy = nx;

    auto minProjection = std::optional<double>{};
    auto maxProjection = std::optional<double>{};
    for (const auto &point : points) {
      const auto distance = std::abs((nx * point.x) + (ny * point.y) - candidate.rho);
      if (distance > config.inlierDistance) {
        continue;
      }

      const auto projection = (dx * point.x) + (dy * point.y);
      if (!minProjection || projection < *minProjection) {
        minProjection = projection;
      }
      if (!maxProjection || projection > *maxProjection) {
        maxProjection = projection;
      }
    }

    if (!minProjection || !maxProjection) {
      continue;
    }

    const auto segmentLength = std::abs(*maxProjection - *minProjection);
    if (segmentLength < config.minSegmentLength) {
      continue;
    }

    const auto startPoint = types::Point{.x = (dx * (*minProjection)) + (nx * candidate.rho),
                                         .y = (dy * (*minProjection)) + (ny * candidate.rho)};
    const auto endPoint = types::Point{.x = (dx * (*maxProjection)) + (nx * candidate.rho),
                                       .y = (dy * (*maxProjection)) + (ny * candidate.rho)};

    lines.push_back(MapLine{types::LineSegment{startPoint, endPoint}, normalized});
  }

  if (lines.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No line segments extracted from Hough candidates."});
  }

  return lines;
}

auto PureEkfLocalizer::reset(const types::Pose &initialPose,
                             const std::array<double, 9> &initialCovariance) -> Status {
  state_ = State{initialPose.x, initialPose.y, initialPose.theta};
  covariance_ = initialCovariance;
  score_ = 0.0;
  hasState_ = true;
  return {};
}

auto PureEkfLocalizer::predict(const types::Twist &control, double dt) -> Status {
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

auto PureEkfLocalizer::update(const types::LidarScan &scan, const types::MapData &map) -> Status {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  if (!signatureMatches(mapSignature_, map)) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Map does not match precomputed line features."});
  }

  if (scan.ranges.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan has no ranges."});
  }

  const auto cosTheta = std::cos(state_.theta);
  const auto sinTheta = std::sin(state_.theta);

  int accepted = 0;
  int tested = 0;

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

    const MapLine *bestLine = nullptr;
    auto bestDistance = std::optional<double>{};
    for (const auto &line : mapLines_) {
      const auto nx = std::cos(line.model.alpha);
      const auto ny = std::sin(line.model.alpha);
      const auto distance = std::abs((nx * mapX) + (ny * mapY) - line.model.rho);
      if (distance > config_.maxAssociationDistance) {
        continue;
      }
      if (!bestDistance || distance < *bestDistance) {
        bestDistance = distance;
        bestLine = &line;
      }
    }

    if (bestLine == nullptr) {
      continue;
    }

    const auto nx = std::cos(bestLine->model.alpha);
    const auto ny = std::sin(bestLine->model.alpha);

    const auto residual = (nx * mapX) + (ny * mapY) - bestLine->model.rho;
    const auto h0 = nx;
    const auto h1 = ny;

    const auto p00 = covariance_[0];
    const auto p01 = covariance_[1];
    const auto p10 = covariance_[3];
    const auto p11 = covariance_[4];
    const auto p20 = covariance_[6];
    const auto p21 = covariance_[7];

    const auto s = (h0 * (p00 * h0 + p01 * h1)) + (h1 * (p10 * h0 + p11 * h1)) +
                   (config_.ekf.measurementNoiseRange * config_.ekf.measurementNoiseRange);
    if (s < kEpsilon) {
      continue;
    }

    ++tested;
    const auto gate = (residual * residual) / s;
    if (gate > config_.gateThreshold) {
      continue;
    }

    const auto ph0 = (p00 * h0) + (p01 * h1);
    const auto ph1 = (p10 * h0) + (p11 * h1);
    const auto ph2 = (p20 * h0) + (p21 * h1);

    const auto kx = ph0 / s;
    const auto ky = ph1 / s;
    const auto kt = ph2 / s;

    state_ = State{state_.x - (kx * residual), state_.y - (ky * residual),
                   util::normalizeAngle(state_.theta - (kt * residual))};

    const auto hP0 = (h0 * p00) + (h1 * p10);
    const auto hP1 = (h0 * p01) + (h1 * p11);
    const auto hP2 = (h0 * covariance_[2]) + (h1 * covariance_[5]);

    covariance_[0] -= kx * hP0;
    covariance_[1] -= kx * hP1;
    covariance_[2] -= kx * hP2;
    covariance_[3] -= ky * hP0;
    covariance_[4] -= ky * hP1;
    covariance_[5] -= ky * hP2;
    covariance_[6] -= kt * hP0;
    covariance_[7] -= kt * hP1;
    covariance_[8] -= kt * hP2;

    ++accepted;
  }

  score_ = tested > 0 ? static_cast<double>(accepted) / static_cast<double>(tested) : 0.0;
  return {};
}

auto PureEkfLocalizer::estimate() const -> Result<LocalizerEstimate> {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  return LocalizerEstimate{.pose = types::Pose{state_.x, state_.y, state_.theta},
                           .covariance = covariance_,
                           .score = score_};
}

} // namespace ad::localization

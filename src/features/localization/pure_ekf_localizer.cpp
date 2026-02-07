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
constexpr double kGateEpsilon = 1e-12;
constexpr double kReferencePoints = 40.0;
constexpr double kMinRangeVarianceFactor = 0.25;
constexpr double kMinAngleVarianceFactor = 0.25;
constexpr double kAngleMseScale = 0.1;

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

struct LineFit {
  ad::localization::LineModel model;
  double alphaRaw;
  std::size_t pointCount;
  double mse;
};

[[nodiscard]] auto fitLine(const std::vector<ad::types::Point> &points) -> std::optional<LineFit> {
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

  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;
  for (const auto &point : points) {
    const auto dx = point.x - meanX;
    const auto dy = point.y - meanY;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }

  if (sxx + syy < kEpsilon) {
    return std::nullopt;
  }

  const auto direction = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  const auto normal = direction + 0.5 * std::numbers::pi;
  const auto nx = std::cos(normal);
  const auto ny = std::sin(normal);
  const auto rho = (nx * meanX) + (ny * meanY);

  auto model = toLineModel(rho, normal);

  const auto lineNx = std::cos(model.alpha);
  const auto lineNy = std::sin(model.alpha);
  double mse = 0.0;
  for (const auto &point : points) {
    const auto distance = (lineNx * point.x) + (lineNy * point.y) - model.rho;
    mse += distance * distance;
  }
  mse /= count;

  return LineFit{.model = model, .alphaRaw = normal, .pointCount = points.size(), .mse = mse};
}

struct LineObservation {
  ad::localization::LineModel observed;
  ad::localization::LineModel expected;
  double nx;
  double ny;
  double rhoSign;
  double rangeVariance;
  double angleVariance;
};

[[nodiscard]] auto makeExpectedLine(const ad::localization::LineModel &mapLine, double x, double y,
                                    double theta) -> LineObservation {
  const auto nx = std::cos(mapLine.alpha);
  const auto ny = std::sin(mapLine.alpha);
  const auto rawRho = mapLine.rho - (nx * x) - (ny * y);
  const auto rawAlpha = ad::localization::util::normalizeAngle(mapLine.alpha - theta);

  auto rhoSign = 1.0;
  auto rho = rawRho;
  auto alpha = rawAlpha;
  if (rho < 0.0) {
    rhoSign = -1.0;
    rho = -rho;
    alpha = ad::localization::util::normalizeAngle(alpha + std::numbers::pi);
  }

  return LineObservation{.observed = ad::localization::LineModel{0.0, 0.0},
                         .expected = ad::localization::LineModel{rho, alpha},
                         .nx = nx,
                         .ny = ny,
                         .rhoSign = rhoSign,
                         .rangeVariance = 0.0,
                         .angleVariance = 0.0};
}

[[nodiscard]] auto gateLineObservation(const LineObservation &observation,
                                       const std::array<double, 9> &covariance, double threshold)
    -> bool {
  const auto h00 = -observation.rhoSign * observation.nx;
  const auto h01 = -observation.rhoSign * observation.ny;

  const auto p00 = covariance[0];
  const auto p01 = covariance[1];
  const auto p02 = covariance[2];
  const auto p10 = covariance[3];
  const auto p11 = covariance[4];
  const auto p12 = covariance[5];
  const auto p22 = covariance[8];

  const auto s00 =
      (h00 * (p00 * h00 + p01 * h01)) + (h01 * (p10 * h00 + p11 * h01)) + observation.rangeVariance;
  const auto s01 = (-h00 * p02) - (h01 * p12);
  const auto s11 = p22 + observation.angleVariance;

  const auto det = (s00 * s11) - (s01 * s01);
  if (std::abs(det) < kGateEpsilon) {
    return false;
  }

  const auto invDet = 1.0 / det;
  const auto inv00 = s11 * invDet;
  const auto inv01 = -s01 * invDet;
  const auto inv11 = s00 * invDet;

  const auto residualRho = observation.observed.rho - observation.expected.rho;
  const auto residualAlpha = ad::localization::util::normalizeAngle(observation.observed.alpha -
                                                                    observation.expected.alpha);
  const auto maha = (residualRho * (inv00 * residualRho + inv01 * residualAlpha)) +
                    (residualAlpha * (inv01 * residualRho + inv11 * residualAlpha));
  return maha <= threshold;
}

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
                                                 .measurementNoiseAngle = 0.12},
                                .maxAssociationDistance = 0.3,
                                .segmentMargin = 0.3,
                                .gateThreshold = 6.0,
                                .minObservations = 3U};
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

    const auto segDx = endPoint.x - startPoint.x;
    const auto segDy = endPoint.y - startPoint.y;
    const auto segLength = std::hypot(segDx, segDy);
    if (segLength < kEpsilon) {
      continue;
    }

    const auto dirX = segDx / segLength;
    const auto dirY = segDy / segLength;
    auto minProjValue = (dirX * startPoint.x) + (dirY * startPoint.y);
    auto maxProjValue = (dirX * endPoint.x) + (dirY * endPoint.y);
    if (minProjValue > maxProjValue) {
      std::swap(minProjValue, maxProjValue);
    }

    lines.push_back(MapLine{types::LineSegment{startPoint, endPoint}, normalized, dirX, dirY,
                            minProjValue, maxProjValue});
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

  auto observations = std::vector<LineObservation>{};
  observations.reserve(mapLines_.size());
  int gatePassed = 0;
  int candidates = 0;
  for (std::size_t lineIndex = 0; lineIndex < mapLines_.size(); ++lineIndex) {
    const auto &bucket = buckets[lineIndex];
    if (bucket.size() < 2U) {
      continue;
    }

    const auto fit = fitLine(bucket);
    if (!fit) {
      continue;
    }

    auto observation =
        makeExpectedLine(mapLines_[lineIndex].model, state_.x, state_.y, state_.theta);
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

    if (!gateLineObservation(observation, covariance_, config_.gateThreshold)) {
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

#include "line_feature_ekf_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <vector>

namespace {

constexpr double kEpsilon = 1e-9;

[[nodiscard]] auto normalizeAngle(double angle) -> double {
  angle = std::fmod(angle + std::numbers::pi, 2.0 * std::numbers::pi);
  if (angle < 0.0) {
    angle += 2.0 * std::numbers::pi;
  }
  return angle - std::numbers::pi;
}

[[nodiscard]] auto mapHasConsistentGrid(const ad::types::MapData &map) -> bool {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  return map.grid.size() == expectedCells;
}

[[nodiscard]] auto lineFromPoints(const ad::types::Point &first, const ad::types::Point &second)
    -> std::optional<ad::localization::LineModel> {
  const auto dx = second.x - first.x;
  const auto dy = second.y - first.y;
  const auto length = std::hypot(dx, dy);
  if (length < kEpsilon) {
    return std::nullopt;
  }

  auto nx = -dy / length;
  auto ny = dx / length;
  auto rho = (nx * first.x) + (ny * first.y);
  auto alpha = std::atan2(ny, nx);
  if (rho < 0.0) {
    rho = -rho;
    alpha = normalizeAngle(alpha + std::numbers::pi);
  }

  return ad::localization::LineModel{rho, normalizeAngle(alpha)};
}

[[nodiscard]] auto lineToMapFrame(const ad::localization::LineModel &line,
                                  const ad::types::Pose &pose) -> ad::localization::LineModel {
  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);

  const auto nx = std::cos(line.alpha);
  const auto ny = std::sin(line.alpha);
  const auto nMapX = (cosTheta * nx) - (sinTheta * ny);
  const auto nMapY = (sinTheta * nx) + (cosTheta * ny);

  auto rho = line.rho + (nMapX * pose.x) + (nMapY * pose.y);
  auto alpha = std::atan2(nMapY, nMapX);
  if (rho < 0.0) {
    rho = -rho;
    alpha = normalizeAngle(alpha + std::numbers::pi);
  }

  return {rho, normalizeAngle(alpha)};
}

[[nodiscard]] auto lineToRobotFrame(const ad::localization::LineModel &line,
                                    const ad::types::Pose &pose) -> ad::localization::LineModel {
  const auto cosTheta = std::cos(pose.theta);
  const auto sinTheta = std::sin(pose.theta);

  const auto nx = std::cos(line.alpha);
  const auto ny = std::sin(line.alpha);
  const auto nRobotX = (cosTheta * nx) + (sinTheta * ny);
  const auto nRobotY = (-sinTheta * nx) + (cosTheta * ny);

  auto rho = line.rho - (nx * pose.x) - (ny * pose.y);
  auto alpha = std::atan2(nRobotY, nRobotX);
  if (rho < 0.0) {
    rho = -rho;
    alpha = normalizeAngle(alpha + std::numbers::pi);
  }

  return {rho, normalizeAngle(alpha)};
}

struct RansacResult {
  ad::localization::LineModel line;
  int inliers;
};

[[nodiscard]] auto runRansac(std::span<const ad::types::Point> points,
                             const ad::localization::RansacConfig &config, std::mt19937 &rng)
    -> std::optional<RansacResult> {
  if (points.size() < 2U) {
    return std::nullopt;
  }

  auto best = std::optional<RansacResult>{};
  std::uniform_int_distribution<std::size_t> dist(0U, points.size() - 1U);

  for (int iter = 0; iter < config.iterations; ++iter) {
    const auto indexA = dist(rng);
    const auto indexB = dist(rng);
    if (indexA == indexB) {
      continue;
    }

    const auto candidate = lineFromPoints(points[indexA], points[indexB]);
    if (!candidate) {
      continue;
    }

    const auto nx = std::cos(candidate->alpha);
    const auto ny = std::sin(candidate->alpha);

    int inliers = 0;
    for (const auto &point : points) {
      const auto distance = std::abs((nx * point.x) + (ny * point.y) - candidate->rho);
      if (distance <= config.inlierDistance) {
        ++inliers;
      }
    }

    if (!best || inliers > best->inliers) {
      best = RansacResult{*candidate, inliers};
    }
  }

  if (!best || best->inliers < config.minInliers) {
    return std::nullopt;
  }

  return best;
}

[[nodiscard]] auto collectOccupiedPoints(const ad::types::MapData &map)
    -> std::vector<ad::types::Point> {
  const auto width = static_cast<std::size_t>(map.width);
  const auto height = static_cast<std::size_t>(map.height);
  auto points = std::vector<ad::types::Point>{};

  for (const auto rowIndex : std::views::iota(std::size_t{0}, height)) {
    for (const auto colIndex : std::views::iota(std::size_t{0}, width)) {
      const auto index = (rowIndex * width) + colIndex;
      if (map.grid[index] <= 0) {
        continue;
      }

      const auto xValue = (static_cast<double>(colIndex) + 0.5) * map.resolution;
      const auto yValue = (static_cast<double>(rowIndex) + 0.5) * map.resolution;
      points.push_back(ad::types::Point{.x = xValue, .y = yValue});
    }
  }

  return points;
}

struct HoughCandidate {
  double rho;
  double alpha;
  int votes;
};

} // namespace

namespace ad::localization {

auto LineFeatureEkfLocalizer::defaultConfig() -> LineFeatureLocalizerConfig {
  return LineFeatureLocalizerConfig{
      .hough = HoughConfig{.thetaBins = 180,
                           .rhoBins = 200,
                           .minVotes = 25,
                           .maxLines = 40,
                           .inlierDistance = 0.12,
                           .minSegmentLength = 0.8,
                           .mergeRho = 0.2,
                           .mergeTheta = 0.08},
      .ransac =
          RansacConfig{.iterations = 120, .inlierDistance = 0.12, .minInliers = 30, .seed = 42U},
      .ekf = EkfConfig{.processNoiseTranslation = 0.05,
                       .processNoiseRotation = 0.03,
                       .measurementNoiseRange = 0.08,
                       .measurementNoiseAngle = 0.04},
      .maxMatchDistance = 0.25,
      .maxMatchAngle = 0.12};
}

LineFeatureEkfLocalizer::LineFeatureEkfLocalizer(std::vector<MapLine> mapLines,
                                                 MapSignature signature,
                                                 LineFeatureLocalizerConfig config)
    : config_(config), mapLines_(std::move(mapLines)), mapSignature_(signature),
      pose_{0.0, 0.0, 0.0}, covariance_{}, score_(0.0), hasState_(false), rng_(config.ransac.seed) {
  covariance_.fill(0.0);
}

auto LineFeatureEkfLocalizer::create(const types::MapData &map, LineFeatureLocalizerConfig config)
    -> Result<LineFeatureEkfLocalizer> {
  const auto signature = mapSignatureFromMap(map);
  if (!signature) {
    return tl::make_unexpected(signature.error());
  }

  const auto mapLines = extractLinesFromMap(map, config.hough);
  if (!mapLines) {
    return tl::make_unexpected(mapLines.error());
  }

  return LineFeatureEkfLocalizer{std::move(*mapLines), *signature, config};
}

auto LineFeatureEkfLocalizer::mapSignatureFromMap(const types::MapData &map)
    -> Result<MapSignature> {
  if (!mapHasConsistentGrid(map)) {
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

auto LineFeatureEkfLocalizer::signatureMatches(const MapSignature &signature,
                                               const types::MapData &map) -> bool {
  return signature.width == map.width && signature.height == map.height &&
         signature.resolution == map.resolution && signature.gridSize == map.grid.size();
}

auto LineFeatureEkfLocalizer::extractLinesFromMap(const types::MapData &map,
                                                  const HoughConfig &config)
    -> Result<std::vector<MapLine>> {
  if (!mapHasConsistentGrid(map)) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (config.thetaBins < 2 || config.rhoBins < 2 || config.minVotes <= 0 || config.maxLines <= 0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Hough configuration is invalid."});
  }

  const auto points = collectOccupiedPoints(map);
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

    const auto normalizedAlpha = normalizeAngle(candidate.alpha);
    bool tooClose = false;
    for (const auto &existing : lines) {
      const auto rhoDiff = std::abs(existing.model.rho - candidate.rho);
      const auto alphaDiff = std::abs(normalizeAngle(existing.model.alpha - normalizedAlpha));
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

    auto rho = candidate.rho;
    auto alpha = normalizeAngle(candidate.alpha);
    if (rho < 0.0) {
      rho = -rho;
      alpha = normalizeAngle(alpha + std::numbers::pi);
    }
    const auto model = LineModel{rho, alpha};
    lines.push_back(MapLine{types::LineSegment{startPoint, endPoint}, model});
  }

  if (lines.empty()) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "No line segments extracted from Hough candidates."});
  }

  return lines;
}

auto LineFeatureEkfLocalizer::reset(const types::Pose &initialPose,
                                    const std::array<double, 9> &initialCovariance) -> Status {
  pose_ = initialPose;
  covariance_ = initialCovariance;
  score_ = 0.0;
  hasState_ = true;
  return {};
}

auto LineFeatureEkfLocalizer::predict(const types::Twist &control, double dt) -> Status {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  if (dt <= 0.0) {
    return tl::make_unexpected(Error{ErrorCode::InvalidInput, "Delta time must be positive."});
  }

  const auto cosTheta = std::cos(pose_.theta);
  const auto sinTheta = std::sin(pose_.theta);
  const auto deltaX = control.v * cosTheta * dt;
  const auto deltaY = control.v * sinTheta * dt;
  const auto deltaTheta = control.w * dt;

  pose_ = types::Pose{pose_.x + deltaX, pose_.y + deltaY, normalizeAngle(pose_.theta + deltaTheta)};

  const auto p00 = covariance_[0];
  const auto p01 = covariance_[1];
  const auto p02 = covariance_[2];
  const auto p10 = covariance_[3];
  const auto p11 = covariance_[4];
  const auto p12 = covariance_[5];
  const auto p20 = covariance_[6];
  const auto p21 = covariance_[7];
  const auto p22 = covariance_[8];

  const auto f02 = -control.v * sinTheta * dt;
  const auto f12 = control.v * cosTheta * dt;

  auto pNew = std::array<double, 9>{};
  pNew[0] = p00 + (f02 * p20) + (p02 * f02) + (f02 * p22 * f02);
  pNew[1] = p01 + (f02 * p21) + (f12 * p02) + (f02 * f12 * p22);
  pNew[2] = p02 + (f02 * p22);
  pNew[3] = p10 + (f12 * p20) + (f02 * p12) + (f02 * f12 * p22);
  pNew[4] = p11 + (f12 * p21) + (f12 * p12) + (f12 * f12 * p22);
  pNew[5] = p12 + (f12 * p22);
  pNew[6] = p20 + (p22 * f02);
  pNew[7] = p21 + (p22 * f12);
  pNew[8] = p22;

  const auto qPos = config_.ekf.processNoiseTranslation * dt;
  const auto qRot = config_.ekf.processNoiseRotation * dt;
  pNew[0] += qPos;
  pNew[4] += qPos;
  pNew[8] += qRot;

  covariance_ = pNew;
  return {};
}

auto LineFeatureEkfLocalizer::update(const types::LidarScan &scan, const types::MapData &map)
    -> Status {
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

  auto points = std::vector<types::Point>{};
  points.reserve(scan.ranges.size());
  for (const auto index : std::views::iota(std::size_t{0}, scan.ranges.size())) {
    const auto range = scan.ranges[index];
    if (!(range > 0.0) || range > scan.maxRange) {
      continue;
    }
    const auto angle = scan.minAngle + (scan.angleIncrement * static_cast<double>(index));
    points.push_back(types::Point{.x = range * std::cos(angle), .y = range * std::sin(angle)});
  }

  if (points.size() < 2U) {
    return tl::make_unexpected(
        Error{ErrorCode::EmptyCollection, "Scan did not yield enough points."});
  }

  const auto ransac = runRansac(points, config_.ransac, rng_);
  if (!ransac) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "RANSAC failed to find a dominant line."});
  }

  score_ = static_cast<double>(ransac->inliers) / static_cast<double>(points.size());

  const auto observedLineMap = lineToMapFrame(ransac->line, pose_);

  auto bestMatch = std::optional<MapLine>{};
  auto bestMetric = std::optional<double>{};
  for (const auto &line : mapLines_) {
    const auto rhoDiff = std::abs(line.model.rho - observedLineMap.rho);
    const auto alphaDiff = std::abs(normalizeAngle(line.model.alpha - observedLineMap.alpha));
    if (rhoDiff > config_.maxMatchDistance || alphaDiff > config_.maxMatchAngle) {
      continue;
    }

    const auto metric = rhoDiff + alphaDiff;
    if (!bestMetric || metric < *bestMetric) {
      bestMetric = metric;
      bestMatch.emplace(line);
    }
  }

  if (!bestMatch) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "No matching map line found for RANSAC output."});
  }

  const auto expectedLineRobot = lineToRobotFrame(bestMatch->model, pose_);
  const auto residualRho = ransac->line.rho - expectedLineRobot.rho;
  const auto residualAlpha = normalizeAngle(ransac->line.alpha - expectedLineRobot.alpha);

  const auto nx = std::cos(bestMatch->model.alpha);
  const auto ny = std::sin(bestMatch->model.alpha);

  const auto h00 = -nx;
  const auto h01 = -ny;
  const auto h02 = 0.0;
  const auto h10 = 0.0;
  const auto h11 = 0.0;
  const auto h12 = -1.0;

  const auto p00 = covariance_[0];
  const auto p01 = covariance_[1];
  const auto p02 = covariance_[2];
  const auto p10 = covariance_[3];
  const auto p11 = covariance_[4];
  const auto p12 = covariance_[5];
  const auto p20 = covariance_[6];
  const auto p21 = covariance_[7];
  const auto p22 = covariance_[8];

  const auto s00 = (h00 * (p00 * h00 + p01 * h01)) + (h01 * (p10 * h00 + p11 * h01)) +
                   (config_.ekf.measurementNoiseRange * config_.ekf.measurementNoiseRange);
  const auto s01 = (-h00 * p02) - (h01 * p12);
  const auto s11 = p22 + (config_.ekf.measurementNoiseAngle * config_.ekf.measurementNoiseAngle);

  const auto det = (s00 * s11) - (s01 * s01);
  if (std::abs(det) < kEpsilon) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "EKF update failed due to singular covariance."});
  }

  const auto invDet = 1.0 / det;

  const auto ph0x = (p00 * h00) + (p01 * h01);
  const auto ph0y = (p10 * h00) + (p11 * h01);
  const auto ph0t = (p20 * h00) + (p21 * h01);

  const auto ph1x = -p02;
  const auto ph1y = -p12;
  const auto ph1t = -p22;

  const auto k0x = ((ph0x * s11) + (ph1x * -s01)) * invDet;
  const auto k0y = ((ph0y * s11) + (ph1y * -s01)) * invDet;
  const auto k0t = ((ph0t * s11) + (ph1t * -s01)) * invDet;

  const auto k1x = ((ph0x * -s01) + (ph1x * s00)) * invDet;
  const auto k1y = ((ph0y * -s01) + (ph1y * s00)) * invDet;
  const auto k1t = ((ph0t * -s01) + (ph1t * s00)) * invDet;

  const auto deltaX = (k0x * residualRho) + (k1x * residualAlpha);
  const auto deltaY = (k0y * residualRho) + (k1y * residualAlpha);
  const auto deltaTheta = (k0t * residualRho) + (k1t * residualAlpha);

  pose_ = types::Pose{pose_.x + deltaX, pose_.y + deltaY, normalizeAngle(pose_.theta + deltaTheta)};

  const auto kh00 = (k0x * h00) + (k1x * h10);
  const auto kh01 = (k0x * h01) + (k1x * h11);
  const auto kh02 = (k0x * h02) + (k1x * h12);
  const auto kh10 = (k0y * h00) + (k1y * h10);
  const auto kh11 = (k0y * h01) + (k1y * h11);
  const auto kh12 = (k0y * h02) + (k1y * h12);
  const auto kh20 = (k0t * h00) + (k1t * h10);
  const auto kh21 = (k0t * h01) + (k1t * h11);
  const auto kh22 = (k0t * h02) + (k1t * h12);

  const auto i00 = 1.0 - kh00;
  const auto i01 = -kh01;
  const auto i02 = -kh02;
  const auto i10 = -kh10;
  const auto i11 = 1.0 - kh11;
  const auto i12 = -kh12;
  const auto i20 = -kh20;
  const auto i21 = -kh21;
  const auto i22 = 1.0 - kh22;

  covariance_[0] = (i00 * p00) + (i01 * p10) + (i02 * p20);
  covariance_[1] = (i00 * p01) + (i01 * p11) + (i02 * p21);
  covariance_[2] = (i00 * p02) + (i01 * p12) + (i02 * p22);
  covariance_[3] = (i10 * p00) + (i11 * p10) + (i12 * p20);
  covariance_[4] = (i10 * p01) + (i11 * p11) + (i12 * p21);
  covariance_[5] = (i10 * p02) + (i11 * p12) + (i12 * p22);
  covariance_[6] = (i20 * p00) + (i21 * p10) + (i22 * p20);
  covariance_[7] = (i20 * p01) + (i21 * p11) + (i22 * p21);
  covariance_[8] = (i20 * p02) + (i21 * p12) + (i22 * p22);

  return {};
}

auto LineFeatureEkfLocalizer::estimate() const -> Result<LocalizerEstimate> {
  if (!hasState_) {
    return tl::make_unexpected(
        Error{ErrorCode::InvalidInput, "Localizer state is not initialized."});
  }

  return LocalizerEstimate{.pose = pose_, .covariance = covariance_, .score = score_};
}

} // namespace ad::localization

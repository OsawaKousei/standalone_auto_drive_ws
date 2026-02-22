#include "ransac_core.hpp"

#include "line_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace {

constexpr double kEpsilon = 1e-9;

auto lineFromTwoPoints(const ad::types::Point &point1, const ad::types::Point &point2)
    -> std::optional<ad::localization::util::LineModel> {
  const auto deltaX = point2.x - point1.x;
  const auto deltaY = point2.y - point1.y;
  const auto norm = std::hypot(deltaX, deltaY);
  if (norm < kEpsilon) {
    return std::nullopt;
  }

  const auto normalX = -deltaY / norm;
  const auto normalY = deltaX / norm;
  const auto rho = (normalX * point1.x) + (normalY * point1.y);
  const auto alpha = std::atan2(normalY, normalX);
  return ad::localization::util::toLineModel(
      ad::localization::util::LineModel{.rho = rho, .alpha = alpha});
}

auto collectInlierIndices(const std::vector<ad::types::Point> &points,
                          const ad::localization::util::LineModel &model, double inlierDistance)
    -> std::vector<std::size_t> {
  const auto normalX = std::cos(model.alpha);
  const auto normalY = std::sin(model.alpha);

  auto inlierIndices = std::vector<std::size_t>{};
  inlierIndices.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto &point = points[index];
    const auto distance = std::abs((normalX * point.x) + (normalY * point.y) - model.rho);
    if (distance <= inlierDistance) {
      inlierIndices.push_back(index);
    }
  }

  return inlierIndices;
}

auto computeProjectionSpan(const std::vector<ad::types::Point> &points,
                           const std::vector<std::size_t> &indices,
                           const ad::localization::util::LineModel &model) -> double {
  if (indices.empty()) {
    return 0.0;
  }

  const auto tangentX = -std::sin(model.alpha);
  const auto tangentY = std::cos(model.alpha);

  auto minProjection = std::numeric_limits<double>::infinity();
  auto maxProjection = -std::numeric_limits<double>::infinity();
  for (const auto index : indices) {
    const auto &point = points[index];
    const auto projection = (tangentX * point.x) + (tangentY * point.y);
    minProjection = std::min(minProjection, projection);
    maxProjection = std::max(maxProjection, projection);
  }

  if (!std::isfinite(minProjection) || !std::isfinite(maxProjection)) {
    return 0.0;
  }
  return std::max(0.0, maxProjection - minProjection);
}

auto drawUniqueSample(std::mt19937 &generator, std::size_t pointCount, std::size_t sampleSize)
    -> std::vector<std::size_t> {
  auto sample = std::vector<std::size_t>{};
  sample.reserve(sampleSize);
  std::uniform_int_distribution<std::size_t> distribution(0U, pointCount - 1U);

  while (sample.size() < sampleSize) {
    const auto candidate = distribution(generator);
    const auto exists = std::find(sample.begin(), sample.end(), candidate) != sample.end();
    if (!exists) {
      sample.push_back(candidate);
    }
  }

  return sample;
}

auto transformScanLineToMap(const ad::localization::util::LineModel &scanLine,
                            const ad::types::Pose &pose) -> ad::localization::util::LineModel {
  return ad::localization::line_geometry::transformLineModelLocalToMap(scanLine, pose);
}

auto sampleTwoDistinct(std::mt19937 &generator, std::size_t count)
    -> std::optional<std::pair<std::size_t, std::size_t>> {
  if (count < 2U) {
    return std::nullopt;
  }

  std::uniform_int_distribution<std::size_t> distribution(0U, count - 1U);
  const auto index0 = distribution(generator);
  auto index1 = distribution(generator);
  if (index0 == index1) {
    return std::nullopt;
  }
  return std::pair<std::size_t, std::size_t>{index0, index1};
}

auto estimatePoseFromTwoPairs(const ad::localization::ransac::LinePairCandidate &pair0,
                              const ad::localization::ransac::LinePairCandidate &pair1)
    -> std::optional<ad::types::Pose> {
  if (pair0.scanLineIndex == pair1.scanLineIndex || pair0.mapLineIndex == pair1.mapLineIndex) {
    return std::nullopt;
  }

  const auto theta0 =
      ad::localization::util::normalizeAngle(pair0.mapLine.alpha - pair0.scanLine.alpha);
  const auto theta1 =
      ad::localization::util::normalizeAngle(pair1.mapLine.alpha - pair1.scanLine.alpha);
  const auto theta =
      std::atan2(std::sin(theta0) + std::sin(theta1), std::cos(theta0) + std::cos(theta1));

  auto matrix00 = 0.0;
  auto matrix01 = 0.0;
  auto matrix11 = 0.0;
  auto rhs0 = 0.0;
  auto rhs1 = 0.0;

  const auto addEquation = [&](const ad::localization::util::LineModel &scanLine,
                               const ad::localization::util::LineModel &mapLine) {
    const auto alpha = ad::localization::util::normalizeAngle(scanLine.alpha + theta);
    const auto normalX = std::cos(alpha);
    const auto normalY = std::sin(alpha);
    const auto rhs = mapLine.rho - scanLine.rho;

    matrix00 += normalX * normalX;
    matrix01 += normalX * normalY;
    matrix11 += normalY * normalY;
    rhs0 += normalX * rhs;
    rhs1 += normalY * rhs;
  };

  addEquation(pair0.scanLine, pair0.mapLine);
  addEquation(pair1.scanLine, pair1.mapLine);

  const auto determinant = (matrix00 * matrix11) - (matrix01 * matrix01);
  if (std::abs(determinant) < kEpsilon) {
    return std::nullopt;
  }

  const auto poseX = ((matrix11 * rhs0) - (matrix01 * rhs1)) / determinant;
  const auto poseY = ((matrix00 * rhs1) - (matrix01 * rhs0)) / determinant;
  if (!std::isfinite(poseX) || !std::isfinite(poseY) || !std::isfinite(theta)) {
    return std::nullopt;
  }

  return ad::types::Pose{.x = poseX, .y = poseY, .theta = theta};
}

auto collectLinePairInliers(
    const ad::types::Pose &hypothesisPose,
    const std::vector<ad::localization::ransac::LinePairCandidate> &candidates,
    const ad::localization::ransac::LinePairRansacConfig &config)
    -> std::vector<ad::localization::ransac::LinePairMatch> {
  auto matches = std::vector<ad::localization::ransac::LinePairMatch>{};
  matches.reserve(candidates.size());

  for (const auto &candidate : candidates) {
    const auto transformed = transformScanLineToMap(candidate.scanLine, hypothesisPose);

    const auto angleResidual = std::abs(
        ad::localization::util::normalizeAngle(transformed.alpha - candidate.mapLine.alpha));
    if (angleResidual > config.inlierAngleThreshold) {
      continue;
    }

    const auto rhoResidual = std::abs(transformed.rho - candidate.mapLine.rho);
    if (rhoResidual > config.inlierRhoThreshold) {
      continue;
    }

    const auto score =
        angleResidual + (rhoResidual / std::max(config.inlierRhoThreshold, kEpsilon));
    matches.push_back(
        ad::localization::ransac::LinePairMatch{.scanLineIndex = candidate.scanLineIndex,
                                                .mapLineIndex = candidate.mapLineIndex,
                                                .angleResidual = angleResidual,
                                                .rhoResidual = rhoResidual,
                                                .score = score});
  }

  std::sort(matches.begin(), matches.end(),
            [](const auto &left, const auto &right) { return left.score < right.score; });

  auto selected = std::vector<ad::localization::ransac::LinePairMatch>{};
  auto usedScan = std::vector<std::size_t>{};
  auto usedMap = std::vector<std::size_t>{};
  usedScan.reserve(matches.size());
  usedMap.reserve(matches.size());

  for (const auto &match : matches) {
    const auto scanAlreadyUsed =
        std::find(usedScan.begin(), usedScan.end(), match.scanLineIndex) != usedScan.end();
    if (scanAlreadyUsed) {
      continue;
    }
    const auto mapAlreadyUsed =
        std::find(usedMap.begin(), usedMap.end(), match.mapLineIndex) != usedMap.end();
    if (mapAlreadyUsed) {
      continue;
    }

    usedScan.push_back(match.scanLineIndex);
    usedMap.push_back(match.mapLineIndex);
    selected.push_back(match);
  }

  return selected;
}

auto meanMatchScore(const std::vector<ad::localization::ransac::LinePairMatch> &inliers) -> double {
  if (inliers.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  auto sum = 0.0;
  for (const auto &inlier : inliers) {
    sum += inlier.score;
  }
  return sum / static_cast<double>(inliers.size());
}

auto passesPosePrior(const ad::types::Pose &hypothesis,
                     const ad::localization::ransac::LinePairRansacConfig &config) -> bool {
  if (!config.usePosePrior) {
    return true;
  }

  const auto dx = hypothesis.x - config.priorPoseX;
  const auto dy = hypothesis.y - config.priorPoseY;
  const auto translationDelta = std::hypot(dx, dy);
  if (translationDelta > config.maxTranslationDelta) {
    return false;
  }

  const auto rotationDelta =
      std::abs(ad::localization::util::normalizeAngle(hypothesis.theta - config.priorPoseTheta));
  return rotationDelta <= config.maxRotationDelta;
}

auto isBetterHypothesis(
    const std::vector<ad::localization::ransac::LinePairMatch> &candidateInliers,
    double candidateMeanScore,
    const std::vector<ad::localization::ransac::LinePairMatch> &bestInliers, double bestMeanScore)
    -> bool {
  if (candidateInliers.size() != bestInliers.size()) {
    return candidateInliers.size() > bestInliers.size();
  }
  return candidateMeanScore < bestMeanScore;
}

} // namespace

namespace ad::localization::ransac {

auto runGenericPointRansac(const std::vector<types::Point> &points,
                           const GenericRansacConfig &config, const InlierIndexSelector &selector,
                           std::uint32_t randomSeed) -> std::optional<GenericRansacResult> {
  if (points.size() < config.sampleSize || config.ransac.maxIterations <= 0 ||
      config.sampleSize < 2U || config.ransac.minInliers < config.sampleSize ||
      config.ransac.minInlierRatio <= 0.0 || config.ransac.minInlierRatio > 1.0 || !selector) {
    return std::nullopt;
  }

  const auto seed =
      randomSeed == 0U ? static_cast<std::uint32_t>(points.size() * 2654435761U) : randomSeed;
  std::mt19937 generator(seed);

  auto bestResult = std::optional<GenericRansacResult>{};
  for (int iteration = 0; iteration < config.ransac.maxIterations; ++iteration) {
    const auto sampleIndices = drawUniqueSample(generator, points.size(), config.sampleSize);
    const auto inlierIndices = selector(points, sampleIndices);
    if (!inlierIndices) {
      continue;
    }

    if (inlierIndices->size() < config.ransac.minInliers) {
      continue;
    }

    const auto inlierRatio =
        static_cast<double>(inlierIndices->size()) / static_cast<double>(points.size());
    if (inlierRatio < config.ransac.minInlierRatio) {
      continue;
    }

    if (!bestResult || inlierIndices->size() > bestResult->size()) {
      bestResult = *inlierIndices;
    }
  }

  return bestResult;
}

auto fitLineToPoints(const std::vector<types::Point> &points, const PointLineRansacConfig &config,
                     std::uint32_t randomSeed) -> std::optional<RansacLineFitResult> {
  if (points.size() < 2U || config.maxIterations <= 0 || !(config.inlierDistance > 0.0) ||
      config.minInliers < 2U || config.minInlierRatio <= 0.0 || config.minInlierRatio > 1.0 ||
      config.minInlierSpan <= 0.0) {
    return std::nullopt;
  }

  const auto genericResult = runGenericPointRansac(
      points,
      GenericRansacConfig{.ransac = RansacConfig{.maxIterations = config.maxIterations,
                                                 .minInliers = config.minInliers,
                                                 .minInlierRatio = config.minInlierRatio},
                          .sampleSize = 2U},
      [&](const std::vector<types::Point> &allPoints, const std::vector<std::size_t> &sampleIndices)
          -> std::optional<std::vector<std::size_t>> {
        if (sampleIndices.size() != 2U) {
          return std::nullopt;
        }
        const auto model =
            lineFromTwoPoints(allPoints[sampleIndices[0]], allPoints[sampleIndices[1]]);
        if (!model) {
          return std::nullopt;
        }
        auto inlierIndices = collectInlierIndices(allPoints, *model, config.inlierDistance);
        const auto inlierSpan = computeProjectionSpan(allPoints, inlierIndices, *model);
        if (inlierSpan < config.minInlierSpan) {
          return std::nullopt;
        }
        return inlierIndices;
      },
      randomSeed);

  if (!genericResult) {
    return std::nullopt;
  }

  auto bestFit = std::optional<RansacLineFitResult>{};
  const auto &indices = *genericResult;
  if (indices.empty()) {
    return std::nullopt;
  }

  auto inliers = std::vector<types::Point>{};
  inliers.reserve(indices.size());
  for (const auto index : indices) {
    inliers.push_back(points[index]);
  }

  const auto fit = util::fitLine(inliers);
  if (!fit) {
    return std::nullopt;
  }

  const auto inlierSpan = computeProjectionSpan(points, indices, fit->model);
  bestFit = RansacLineFitResult{.fit = *fit,
                                .inlierCount = inliers.size(),
                                .inlierSpan = inlierSpan,
                                .inlierIndices = indices};
  return bestFit;
}

auto runLinePairRansacWithDiagnostics(const std::vector<LinePairCandidate> &candidates,
                                      const LinePairRansacConfig &config, std::uint32_t randomSeed)
    -> LinePairRansacResult {
  auto result = LinePairRansacResult{};
  result.diagnostics.iterationsRequested = config.maxIterations;

  if (config.maxIterations <= 0 || config.minInliers < 2U || config.minInlierRatio <= 0.0 ||
      config.minInlierRatio > 1.0 || config.inlierAngleThreshold <= 0.0 ||
      config.inlierRhoThreshold <= 0.0 || config.maxTranslationDelta < 0.0 ||
      config.maxRotationDelta < 0.0 || config.maxMeanResidual <= 0.0) {
    result.diagnostics.configurationValid = false;
    return result;
  }

  if (candidates.size() < 2U || config.lineCountForRatio == 0U) {
    return result;
  }

  const auto seed =
      randomSeed == 0U ? static_cast<std::uint32_t>(candidates.size() * 2654435761U) : randomSeed;
  auto generator = std::mt19937(seed);
  auto bestMeanScore = std::numeric_limits<double>::infinity();

  for (int iteration = 0; iteration < config.maxIterations; ++iteration) {
    const auto sample = sampleTwoDistinct(generator, candidates.size());
    if (!sample) {
      ++result.diagnostics.duplicateSampleRejects;
      continue;
    }

    const auto poseHypothesis =
        estimatePoseFromTwoPairs(candidates[sample->first], candidates[sample->second]);
    if (!poseHypothesis) {
      ++result.diagnostics.hypothesisRejects;
      continue;
    }

    if (!passesPosePrior(*poseHypothesis, config)) {
      ++result.diagnostics.posePriorRejects;
      continue;
    }

    auto inliers = collectLinePairInliers(*poseHypothesis, candidates, config);
    if (inliers.size() < config.minInliers) {
      ++result.diagnostics.minInlierRejects;
      continue;
    }

    const auto inlierRatio =
        static_cast<double>(inliers.size()) / static_cast<double>(config.lineCountForRatio);
    if (inlierRatio < config.minInlierRatio) {
      ++result.diagnostics.ratioRejects;
      continue;
    }

    const auto currentMeanScore = meanMatchScore(inliers);
    if (currentMeanScore > config.maxMeanResidual) {
      ++result.diagnostics.residualRejects;
      continue;
    }

    ++result.diagnostics.acceptedHypotheses;

    if (isBetterHypothesis(inliers, currentMeanScore, result.inliers, bestMeanScore)) {
      bestMeanScore = currentMeanScore;
      result.inliers = std::move(inliers);
    }
  }

  result.diagnostics.bestInlierCount = result.inliers.size();
  return result;
}

auto runLinePairRansac(const std::vector<LinePairCandidate> &candidates,
                       const LinePairRansacConfig &config, std::uint32_t randomSeed)
    -> std::vector<LinePairMatch> {
  return runLinePairRansacWithDiagnostics(candidates, config, randomSeed).inliers;
}

} // namespace ad::localization::ransac

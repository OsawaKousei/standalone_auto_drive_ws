#pragma once

#include "../localizer_util.hpp"
#include "ransac_config.hpp"

#include "../../../shared/types.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace ad::localization::ransac {

struct GenericRansacConfig {
  int maxIterations;
  std::size_t sampleSize;
  std::size_t minInliers;
  double minInlierRatio;
};

struct GenericRansacResult {
  std::vector<std::size_t> inlierIndices;
};

using InlierIndexSelector = std::function<std::optional<std::vector<std::size_t>>(
    const std::vector<types::Point> &points, const std::vector<std::size_t> &sampleIndices)>;

[[nodiscard]] auto
runGenericPointRansac(const std::vector<types::Point> &points, const GenericRansacConfig &config,
                      const InlierIndexSelector &selector, std::uint32_t randomSeed = 0U)
    -> std::optional<GenericRansacResult>;

struct RansacLineFitResult {
  util::LineFit fit;
  std::size_t inlierCount;
  double inlierSpan;
  std::vector<std::size_t> inlierIndices;
};

struct PointLineRansacConfig {
  int maxIterations;
  double inlierDistance;
  std::size_t minInliers;
  double minInlierRatio;
  double minInlierSpan;
};

[[nodiscard]] auto fitLineToPoints(const std::vector<types::Point> &points,
                                   const PointLineRansacConfig &config,
                                   std::uint32_t randomSeed = 0U)
    -> std::optional<RansacLineFitResult>;

struct LinePairCandidate {
  std::size_t scanLineIndex;
  std::size_t mapLineIndex;
  util::LineModel scanLine;
  util::LineModel mapLine;
};

struct LinePairMatch {
  std::size_t scanLineIndex;
  std::size_t mapLineIndex;
  double angleResidual;
  double rhoResidual;
  double score;
};

struct LinePairRansacConfig {
  int maxIterations;
  std::size_t minInliers;
  double minInlierRatio;
  double inlierAngleThreshold;
  double inlierRhoThreshold;
  std::size_t lineCountForRatio;
};

struct LinePairRansacDiagnostics {
  bool configurationValid = true;
  int iterationsRequested = 0;
  int duplicateSampleRejects = 0;
  int hypothesisRejects = 0;
  int minInlierRejects = 0;
  int ratioRejects = 0;
  int acceptedHypotheses = 0;
  std::size_t bestInlierCount = 0U;
};

struct LinePairRansacResult {
  std::vector<LinePairMatch> inliers;
  LinePairRansacDiagnostics diagnostics;
};

[[nodiscard]] auto
runLinePairRansacWithDiagnostics(const std::vector<LinePairCandidate> &candidates,
                                 const LinePairRansacConfig &config, std::uint32_t randomSeed = 0U)
    -> LinePairRansacResult;

[[nodiscard]] auto runLinePairRansac(const std::vector<LinePairCandidate> &candidates,
                                     const LinePairRansacConfig &config,
                                     std::uint32_t randomSeed = 0U) -> std::vector<LinePairMatch>;

} // namespace ad::localization::ransac

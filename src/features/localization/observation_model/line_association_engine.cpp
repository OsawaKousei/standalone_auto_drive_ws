#include "line_association_engine.hpp"

#include "line_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ad::localization::line_association_engine {

auto buildCandidatePairs(const std::vector<util::MapLine> &scanLines,
                         const std::vector<util::MapLine> &mapLines,
                         const types::Pose &predictedPose, const CandidatePairBuildConfig &config)
    -> std::vector<ransac::LinePairCandidate> {
  auto candidates = std::vector<ransac::LinePairCandidate>{};
  const auto angleGate = std::max(config.candidateAngleGateMin, config.mergeTheta * 2.0);
  const auto rhoGate = std::max(config.candidateRhoGateMin, config.maxAssociationDistance * 2.0);

  for (std::size_t scanIndex = 0; scanIndex < scanLines.size(); ++scanIndex) {
    const auto predictedMapLine =
        line_geometry::transformLineModelLocalToMap(scanLines[scanIndex].model, predictedPose);
    const auto scanStart = scanLines[scanIndex].segment.start;
    const auto scanEnd = scanLines[scanIndex].segment.end;
    const auto cosTheta = std::cos(predictedPose.theta);
    const auto sinTheta = std::sin(predictedPose.theta);
    const auto scanStartMap =
        types::Point{.x = predictedPose.x + (cosTheta * scanStart.x) - (sinTheta * scanStart.y),
                     .y = predictedPose.y + (sinTheta * scanStart.x) + (cosTheta * scanStart.y)};
    const auto scanEndMap =
        types::Point{.x = predictedPose.x + (cosTheta * scanEnd.x) - (sinTheta * scanEnd.y),
                     .y = predictedPose.y + (sinTheta * scanEnd.x) + (cosTheta * scanEnd.y)};

    for (std::size_t mapIndex = 0; mapIndex < mapLines.size(); ++mapIndex) {
      const auto angleResidual =
          std::abs(util::normalizeAngle(predictedMapLine.alpha - mapLines[mapIndex].model.alpha));
      if (angleResidual > angleGate) {
        continue;
      }

      const auto rhoResidual = std::abs(predictedMapLine.rho - mapLines[mapIndex].model.rho);
      if (rhoResidual > rhoGate) {
        continue;
      }

      const auto scanProjection0 = (mapLines[mapIndex].directionX * scanStartMap.x) +
                                   (mapLines[mapIndex].directionY * scanStartMap.y);
      const auto scanProjection1 = (mapLines[mapIndex].directionX * scanEndMap.x) +
                                   (mapLines[mapIndex].directionY * scanEndMap.y);
      const auto scanProjectionMin = std::min(scanProjection0, scanProjection1);
      const auto scanProjectionMax = std::max(scanProjection0, scanProjection1);
      const auto mapProjectionMin = mapLines[mapIndex].minProjection - config.segmentMargin;
      const auto mapProjectionMax = mapLines[mapIndex].maxProjection + config.segmentMargin;
      const auto overlapMin = std::max(scanProjectionMin, mapProjectionMin);
      const auto overlapMax = std::min(scanProjectionMax, mapProjectionMax);
      if (overlapMax < overlapMin) {
        continue;
      }

      candidates.push_back(ransac::LinePairCandidate{.scanLineIndex = scanIndex,
                                                     .mapLineIndex = mapIndex,
                                                     .scanLine = scanLines[scanIndex].model,
                                                     .mapLine = mapLines[mapIndex].model});
    }
  }

  return candidates;
}

auto countDistinctScanLines(const std::vector<ransac::LinePairCandidate> &candidates)
    -> std::size_t {
  auto scanIndices = std::vector<std::size_t>{};
  scanIndices.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    scanIndices.push_back(candidate.scanLineIndex);
  }
  std::sort(scanIndices.begin(), scanIndices.end());
  const auto uniqueEnd = std::unique(scanIndices.begin(), scanIndices.end());
  return static_cast<std::size_t>(std::distance(scanIndices.begin(), uniqueEnd));
}

} // namespace ad::localization::line_association_engine

#pragma once

#include "ransac_core.hpp"

#include "../localizer_util.hpp"

#include "../../../shared/types.hpp"

#include <vector>

namespace ad::localization::line_association_engine {

struct CandidatePairBuildConfig {
  double candidateAngleGateMin;
  double candidateRhoGateMin;
  double mergeTheta;
  double maxAssociationDistance;
  double segmentMargin;
};

[[nodiscard]] auto buildCandidatePairs(const std::vector<util::MapLine> &scanLines,
                                       const std::vector<util::MapLine> &mapLines,
                                       const types::Pose &predictedPose,
                                       const CandidatePairBuildConfig &config)
    -> std::vector<ransac::LinePairCandidate>;

[[nodiscard]] auto countDistinctScanLines(const std::vector<ransac::LinePairCandidate> &candidates)
    -> std::size_t;

} // namespace ad::localization::line_association_engine

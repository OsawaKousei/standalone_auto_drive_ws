#pragma once

#include "../i_observation_model.hpp"
#include "line_extractor.hpp"
#include "line_observation_util.hpp"

#include <memory>
#include <vector>

namespace ad::localization {

struct RansacLineAssociationModelConfig {
  const line_extractor::MapLineExtractionConfig mapLineExtraction;
  const double measurementNoiseRange;
  const double measurementNoiseAngle;
  const std::size_t minObservations;

  const double pointDistanceThreshold;
  const std::size_t minInlierPoints;
  const int pointRansacMaxIterations;
  const int maxExtractedScanLines;
  const double minExtractedSegmentLength;
  const std::size_t minRemainingPoints;
  const int sampleNeighborWindow;
  const int maxContinuityGap;

  const int translationRansacMaxIterations;
  const double lineAngleThreshold;
  const double lineRhoThreshold;
  const double parallelRejectThreshold;
  const std::size_t minPoseInliers;
  const double segmentMargin;

  const double contextGateThreshold;

  const bool useEkfGate;
  const double gateThreshold;
};

class RansacLineAssociationModel final : public IObservationModel {
public:
  [[nodiscard]] static auto create(const types::MapData &map,
                                   RansacLineAssociationModelConfig config)
      -> Result<std::unique_ptr<RansacLineAssociationModel>>;

  RansacLineAssociationModel(std::vector<observation_model::util::MapLine> mapLines,
                             observation_model::util::MapSignature signature,
                             RansacLineAssociationModelConfig config);

  [[nodiscard]] auto buildUpdateInput(const types::LidarScan &scan, const types::MapData &map,
                                      const types::Pose &predictedPose,
                                      const CovarianceMatrix &predictedCovariance) const
      -> Result<std::optional<ObservationUpdateInput>> override;

private:
  RansacLineAssociationModelConfig config_;
  std::vector<observation_model::util::MapLine> mapLines_{};
  observation_model::util::MapSignature mapSignature_;
};

} // namespace ad::localization

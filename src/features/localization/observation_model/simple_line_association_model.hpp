#pragma once

#include "../i_observation_model.hpp"
#include "line_extractor.hpp"
#include "line_observation_util.hpp"

#include <memory>
#include <vector>

namespace ad::localization {

struct SimpleLineAssociationModelConfig {
  const line_extractor::MapLineExtractionConfig mapLineExtraction;
  const double measurementNoiseRange;
  const double measurementNoiseAngle;
  const double maxAssociationDistance;
  const double segmentMargin;
  const double gateThreshold;
  const std::size_t minObservations;
};

class SimpleLineAssociationModel final : public IObservationModel {
public:
  [[nodiscard]] static auto create(const types::MapData &map,
                                   SimpleLineAssociationModelConfig config)
      -> Result<std::unique_ptr<SimpleLineAssociationModel>>;

  SimpleLineAssociationModel(std::vector<observation_model::util::MapLine> mapLines,
                             observation_model::util::MapSignature signature,
                             SimpleLineAssociationModelConfig config);

  [[nodiscard]] auto buildUpdateInput(const types::LidarScan &scan, const types::MapData &map,
                                      const types::Pose &predictedPose,
                                      const CovarianceMatrix &predictedCovariance) const
      -> Result<std::optional<ObservationUpdateInput>> override;

private:
  SimpleLineAssociationModelConfig config_;
  std::vector<observation_model::util::MapLine> mapLines_{};
  observation_model::util::MapSignature mapSignature_;
};

} // namespace ad::localization

#pragma once

#include "../i_observation_model.hpp"
#include "../localizer_util.hpp"
#include "ransac_config.hpp"
#include "simple_line_association_model.hpp"

#include <memory>
#include <vector>

namespace ad::localization {

struct RansacLineAssociationModelConfig {
  const SimpleLineAssociationModelConfig baseObservation;
  const RansacLineExtractionConfig ransacLineExtraction;
  const RansacConfig ransac;
};

class RansacLineAssociationModel final : public IObservationModel {
public:
  [[nodiscard]] static auto create(const types::MapData &map,
                                   RansacLineAssociationModelConfig config)
      -> Result<std::unique_ptr<RansacLineAssociationModel>>;

  RansacLineAssociationModel(std::vector<util::MapLine> mapLines, util::MapSignature signature,
                             RansacLineAssociationModelConfig config);

  [[nodiscard]] auto buildUpdateInput(const types::LidarScan &scan, const types::MapData &map,
                                      const types::Pose &predictedPose,
                                      const CovarianceMatrix &predictedCovariance) const
      -> Result<std::optional<ObservationUpdateInput>> override;

private:
  RansacLineAssociationModelConfig config_;
  std::vector<util::MapLine> mapLines_{};
  util::MapSignature mapSignature_;
};

} // namespace ad::localization

#pragma once

#include "i_observation_model.hpp"
#include "localizer_util.hpp"

#include <memory>
#include <vector>

namespace ad::localization {

class HoughObservationModel final : public IObservationModel {
public:
  [[nodiscard]] static auto create(const types::MapData &map, EkfLocalizerConfig config)
      -> Result<std::unique_ptr<HoughObservationModel>>;

  HoughObservationModel(std::vector<util::MapLine> mapLines, util::MapSignature signature,
                        EkfLocalizerConfig config);

  [[nodiscard]] auto buildUpdateInput(const types::LidarScan &scan, const types::MapData &map,
                                      const types::Pose &predictedPose,
                                      const CovarianceMatrix &predictedCovariance) const
      -> Result<std::optional<ObservationUpdateInput>> override;

private:
  EkfLocalizerConfig config_;
  std::vector<util::MapLine> mapLines_{};
  util::MapSignature mapSignature_;
};

} // namespace ad::localization

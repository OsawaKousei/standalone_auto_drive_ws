#pragma once

#include "i_observation_model.hpp"
#include "localizer_util.hpp"

#include "../../shared/text_config.hpp"

#include <memory>
#include <optional>
#include <vector>

namespace ad::localization {

struct HoughObservationModelConfig {
  const HoughConfig hough;
  const double measurementNoiseRange;
  const double measurementNoiseAngle;
  const double maxAssociationDistance;
  const double segmentMargin;
  const double gateThreshold;
  const std::size_t minObservations;
};

class HoughObservationModel final : public IObservationModel {
public:
  [[nodiscard]] static auto create(const types::MapData &map, HoughObservationModelConfig config)
      -> Result<std::unique_ptr<HoughObservationModel>>;
  [[nodiscard]] static auto
  createFromConfig(const types::MapData &map,
                   const std::optional<::ad::config::TextConfig> &configDoc)
      -> Result<std::unique_ptr<HoughObservationModel>>;

  HoughObservationModel(std::vector<util::MapLine> mapLines, util::MapSignature signature,
                        HoughObservationModelConfig config);

  [[nodiscard]] auto buildUpdateInput(const types::LidarScan &scan, const types::MapData &map,
                                      const types::Pose &predictedPose,
                                      const CovarianceMatrix &predictedCovariance) const
      -> Result<std::optional<ObservationUpdateInput>> override;

private:
  HoughObservationModelConfig config_;
  std::vector<util::MapLine> mapLines_{};
  util::MapSignature mapSignature_;
};

} // namespace ad::localization

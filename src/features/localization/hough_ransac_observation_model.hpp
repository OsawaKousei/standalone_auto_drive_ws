#pragma once

#include "hough_observation_model.hpp"
#include "i_observation_model.hpp"
#include "localizer_util.hpp"

#include <memory>
#include <vector>

namespace ad::localization {

struct RansacConfig {
  const int maxIterations;
  const double inlierDistance;
  const std::size_t minInliers;
  const double minInlierRatio;
};

struct HoughRansacObservationModelConfig {
  const HoughObservationModelConfig houghObservation;
  const RansacConfig ransac;
};

class HoughRansacObservationModel final : public IObservationModel {
public:
  [[nodiscard]] static auto create(const types::MapData &map,
                                   HoughRansacObservationModelConfig config)
      -> Result<std::unique_ptr<HoughRansacObservationModel>>;

  HoughRansacObservationModel(std::vector<util::MapLine> mapLines, util::MapSignature signature,
                              HoughRansacObservationModelConfig config);

  [[nodiscard]] auto buildUpdateInput(const types::LidarScan &scan, const types::MapData &map,
                                      const types::Pose &predictedPose,
                                      const CovarianceMatrix &predictedCovariance) const
      -> Result<std::optional<ObservationUpdateInput>> override;

private:
  HoughRansacObservationModelConfig config_;
  std::vector<util::MapLine> mapLines_{};
  util::MapSignature mapSignature_;
};

} // namespace ad::localization

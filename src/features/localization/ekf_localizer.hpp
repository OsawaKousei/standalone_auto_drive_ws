#pragma once

#include "i_localizer.hpp"
#include "i_observation_model.hpp"
#include "localization_config.hpp"

#include <memory>

namespace ad::localization {

class EkfLocalizer final : public ILocalizer {
public:
  [[nodiscard]] static auto create(EkfLocalizerConfig config,
                                   std::unique_ptr<IObservationModel> observationModel)
      -> Result<std::unique_ptr<EkfLocalizer>>;

  EkfLocalizer(EkfLocalizerConfig config, std::unique_ptr<IObservationModel> observationModel);

  [[nodiscard]] auto reset(const types::Pose &initialPose,
                           const CovarianceMatrix &initialCovariance) -> Status override;
  [[nodiscard]] auto predictOdometry(const types::OdometryDelta &delta) -> Status override;
  [[nodiscard]] auto update(const types::LidarScan &scan, const types::MapData &map)
      -> Status override;
  [[nodiscard]] auto estimate() const -> Result<LocalizerEstimate> override;

private:
  struct State {
    double x;
    double y;
    double theta;
  };

  EkfLocalizerConfig config_;
  std::unique_ptr<IObservationModel> observationModel_;
  State state_;
  CovarianceMatrix covariance_{CovarianceMatrix::Zero()};
  double score_ = 0.0;
  bool hasState_ = false;
};

} // namespace ad::localization

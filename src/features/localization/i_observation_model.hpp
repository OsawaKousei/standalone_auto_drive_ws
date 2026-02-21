#pragma once

#include "localization_config.hpp"

#include "../../shared/result.hpp"
#include "../../shared/types.hpp"

#include <Eigen/Dense>
#include <optional>

namespace ad::localization {

struct ObservationUpdateInput {
  Eigen::VectorXd residual;
  Eigen::MatrixXd measurementMatrix;
  Eigen::MatrixXd measurementNoise;
  double score = 0.0;
};

class IObservationModel {
public:
  virtual ~IObservationModel() = default;
  IObservationModel() = default;
  IObservationModel(const IObservationModel &) = delete;
  auto operator=(const IObservationModel &) -> IObservationModel & = delete;
  IObservationModel(IObservationModel &&) = delete;
  auto operator=(IObservationModel &&) -> IObservationModel & = delete;

  [[nodiscard]] virtual auto buildUpdateInput(const types::LidarScan &scan,
                                              const types::MapData &map,
                                              const types::Pose &predictedPose,
                                              const CovarianceMatrix &predictedCovariance) const
      -> Result<std::optional<ObservationUpdateInput>> = 0;
};

} // namespace ad::localization

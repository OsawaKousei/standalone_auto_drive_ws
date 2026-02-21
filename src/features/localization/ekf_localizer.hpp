#pragma once

#include "i_localizer.hpp"
#include "localization_config.hpp"
#include "localizer_util.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace ad::localization {

class EkfLocalizer final : public ILocalizer {
public:
  [[nodiscard]] static auto create(const types::MapData &map, EkfLocalizerConfig config)
      -> Result<std::unique_ptr<EkfLocalizer>>;

  EkfLocalizer(std::vector<util::MapLine> mapLines, util::MapSignature signature,
               EkfLocalizerConfig config);

  [[nodiscard]] auto reset(const types::Pose &initialPose,
                           const CovarianceMatrix &initialCovariance) -> Status override;
  [[nodiscard]] auto predict(const types::Twist &control, double deltaT) -> Status override;
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
  std::vector<util::MapLine> mapLines_{};
  util::MapSignature mapSignature_;
  State state_;
  CovarianceMatrix covariance_{CovarianceMatrix::Zero()};
  double score_ = 0.0;
  bool hasState_ = false;
};

} // namespace ad::localization

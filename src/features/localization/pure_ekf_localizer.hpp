#pragma once

#include "i_localizer.hpp"
#include "localization_config.hpp"

#include <cstddef>
#include <vector>

namespace ad::localization {

struct PureEkfLocalizerConfig {
  const HoughConfig hough;
  const EkfConfig ekf;
  const double maxAssociationDistance;
  const double gateThreshold;
};

class PureEkfLocalizer final : public ILocalizer {
public:
  [[nodiscard]] static auto defaultConfig() -> PureEkfLocalizerConfig;
  [[nodiscard]] static auto create(const types::MapData &map, PureEkfLocalizerConfig config)
      -> Result<PureEkfLocalizer>;

  [[nodiscard]] auto reset(const types::Pose &initialPose,
                           const std::array<double, 9> &initialCovariance) -> Status override;
  [[nodiscard]] auto predict(const types::Twist &control, double dt) -> Status override;
  [[nodiscard]] auto update(const types::LidarScan &scan, const types::MapData &map)
      -> Status override;
  [[nodiscard]] auto estimate() const -> Result<LocalizerEstimate> override;

private:
  struct MapSignature {
    int width;
    int height;
    double resolution;
    std::size_t gridSize;
  };

  struct LineModel {
    double rho;
    double alpha;
  };

  struct MapLine {
    types::LineSegment segment;
    LineModel model;
  };

  PureEkfLocalizer(std::vector<MapLine> mapLines, MapSignature signature,
                   PureEkfLocalizerConfig config);

  [[nodiscard]] static auto mapSignatureFromMap(const types::MapData &map) -> Result<MapSignature>;
  [[nodiscard]] static auto signatureMatches(const MapSignature &signature,
                                             const types::MapData &map) -> bool;
  [[nodiscard]] static auto extractLinesFromMap(const types::MapData &map,
                                                const HoughConfig &config)
      -> Result<std::vector<MapLine>>;

  PureEkfLocalizerConfig config_;
  std::vector<MapLine> mapLines_;
  MapSignature mapSignature_;
  types::Pose pose_;
  std::array<double, 9> covariance_;
  double score_;
  bool hasState_;
};

} // namespace ad::localization

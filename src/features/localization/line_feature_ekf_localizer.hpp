#pragma once

#include "i_localizer.hpp"

#include <cstddef>
#include <random>
#include <vector>

namespace ad::localization {

struct HoughConfig {
  const int thetaBins;
  const int rhoBins;
  const int minVotes;
  const int maxLines;
  const double inlierDistance;
  const double minSegmentLength;
  const double mergeRho;
  const double mergeTheta;
};

struct RansacConfig {
  const int iterations;
  const double inlierDistance;
  const int minInliers;
  const unsigned int seed;
};

struct EkfConfig {
  const double processNoiseTranslation;
  const double processNoiseRotation;
  const double measurementNoiseRange;
  const double measurementNoiseAngle;
};

struct LineFeatureLocalizerConfig {
  const HoughConfig hough;
  const RansacConfig ransac;
  const EkfConfig ekf;
  const double maxMatchDistance;
  const double maxMatchAngle;
};

struct LineModel {
  double rho;
  double alpha;
};

class LineFeatureEkfLocalizer final : public ILocalizer {
public:
  [[nodiscard]] static auto defaultConfig() -> LineFeatureLocalizerConfig;
  [[nodiscard]] static auto create(const types::MapData &map, LineFeatureLocalizerConfig config)
      -> Result<LineFeatureEkfLocalizer>;

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

  struct MapLine {
    types::LineSegment segment;
    LineModel model;
  };

  LineFeatureEkfLocalizer(std::vector<MapLine> mapLines, MapSignature signature,
                          LineFeatureLocalizerConfig config);

  [[nodiscard]] static auto mapSignatureFromMap(const types::MapData &map) -> Result<MapSignature>;
  [[nodiscard]] static auto signatureMatches(const MapSignature &signature,
                                             const types::MapData &map) -> bool;
  [[nodiscard]] static auto extractLinesFromMap(const types::MapData &map,
                                                const HoughConfig &config)
      -> Result<std::vector<MapLine>>;

  LineFeatureLocalizerConfig config_;
  std::vector<MapLine> mapLines_;
  MapSignature mapSignature_;
  types::Pose pose_;
  std::array<double, 9> covariance_;
  double score_;
  bool hasState_;
  std::mt19937 rng_;
};

} // namespace ad::localization

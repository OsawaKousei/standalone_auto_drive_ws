#pragma once

#include "i_localizer.hpp"

namespace ad::localization {

class RansacLocalizer final : public ILocalizer {
public:
  RansacLocalizer() = default;
  [[nodiscard]] auto estimate(std::span<const double> scan, const types::MapData &map,
                              const types::Pose &initialPose) const -> Result<types::Pose> override;
};

} // namespace ad::localization

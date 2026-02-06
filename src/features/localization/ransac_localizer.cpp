#include "ransac_localizer.hpp"

#include <cstddef>

namespace ad::localization {

auto RansacLocalizer::estimate(std::span<const double> scan, const types::MapData &map,
                               const types::Pose &initialPose) const -> Result<types::Pose> {
  const auto expectedCells =
      static_cast<std::size_t>(map.width) * static_cast<std::size_t>(map.height);
  if (map.grid.size() != expectedCells) {
    return tl::make_unexpected(
        Error{ErrorCode::SizeMismatch, "Map grid size does not match width and height."});
  }

  if (scan.empty()) {
    return tl::make_unexpected(Error{ErrorCode::EmptyCollection, "Scan has no points."});
  }

  return initialPose;
}

} // namespace ad::localization

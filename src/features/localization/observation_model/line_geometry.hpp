#pragma once

#include "../localizer_util.hpp"

#include "../../../shared/types.hpp"

#include <optional>

namespace ad::localization::line_geometry {

[[nodiscard]] auto buildMapLineFromSegment(const types::Point &start, const types::Point &end,
                                           double minSegmentLength = 1e-9)
    -> std::optional<util::MapLine>;

[[nodiscard]] auto
buildMapLineFromModelAndProjectionSpan(const util::LineModel &model, double minProjection,
                                       double maxProjection, double minSegmentLength = 1e-9)
    -> std::optional<util::MapLine>;

[[nodiscard]] auto transformLineModelLocalToMap(const util::LineModel &localLine,
                                                const types::Pose &pose) -> util::LineModel;

} // namespace ad::localization::line_geometry

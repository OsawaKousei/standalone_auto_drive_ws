#pragma once

#include "i_localizer.hpp"

#include "../../shared/result.hpp"
#include "../../shared/text_config.hpp"

#include <memory>
#include <optional>
#include <string_view>

namespace ad::localization {

[[nodiscard]] auto
createLocalizerFromConfig(std::string_view algorithm, const types::MapData &map,
                          const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<ILocalizer>>;

[[nodiscard]] auto
parseInitialCovarianceFromConfig(const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<CovarianceMatrix>;

} // namespace ad::localization

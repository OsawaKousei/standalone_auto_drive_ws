#pragma once

#include "i_controller.hpp"

#include "../../shared/result.hpp"
#include "../../shared/text_config.hpp"

#include <memory>
#include <optional>
#include <string_view>

namespace ad::control {

[[nodiscard]] auto
createControllerFromConfig(std::string_view algorithm,
                           const std::optional<::ad::config::TextConfig> &configDoc)
    -> Result<std::unique_ptr<IController>>;

} // namespace ad::control

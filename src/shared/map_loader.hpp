#pragma once

#include "result.hpp"
#include "types.hpp"

#include <string_view>

namespace ad {

[[nodiscard]] auto loadMapFromYaml(std::string_view yamlPath) -> Result<types::MapData>;

} // namespace ad

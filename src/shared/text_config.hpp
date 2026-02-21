#pragma once

#include "result.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace ad::config {

class TextConfig {
public:
  using Section = std::unordered_map<std::string, std::string>;

  [[nodiscard]] auto findRaw(std::string_view section, std::string_view key) const
      -> std::optional<std::string_view>;
  auto setValue(std::string_view section, std::string_view key, std::string value) -> void;

private:
  std::unordered_map<std::string, Section> sections_;
};

[[nodiscard]] auto loadTextConfig(std::string_view path) -> Result<TextConfig>;
[[nodiscard]] auto parseQuotedString(std::string_view raw) -> Result<std::string>;
[[nodiscard]] auto parseDoubleValue(std::string_view raw) -> Result<double>;
[[nodiscard]] auto parseIntValue(std::string_view raw) -> Result<int>;
[[nodiscard]] auto parseBoolValue(std::string_view raw) -> Result<bool>;
[[nodiscard]] auto parseArrayFlat(std::string_view raw) -> Result<std::vector<double>>;

} // namespace ad::config

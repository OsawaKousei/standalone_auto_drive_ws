#include "text_config.hpp"

#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ad::config {

namespace {

[[nodiscard]] auto trim(std::string_view value) -> std::string_view {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string_view::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

[[nodiscard]] auto stripComment(std::string_view line) -> std::string {
  auto output = std::string{};
  auto quote = char{0};
  for (const auto character : line) {
    if (quote == 0 && (character == '"' || character == '\'')) {
      quote = character;
      output.push_back(character);
      continue;
    }
    if (quote != 0 && character == quote) {
      quote = 0;
      output.push_back(character);
      continue;
    }
    if (quote == 0 && character == '#') {
      break;
    }
    output.push_back(character);
  }
  return output;
}

[[nodiscard]] auto bracketDepthDelta(std::string_view value) -> int {
  auto depth = 0;
  auto quote = char{0};
  for (const auto character : value) {
    if (quote == 0 && (character == '"' || character == '\'')) {
      quote = character;
      continue;
    }
    if (quote != 0 && character == quote) {
      quote = 0;
      continue;
    }
    if (quote != 0) {
      continue;
    }
    if (character == '[') {
      ++depth;
    } else if (character == ']') {
      --depth;
    }
  }
  return depth;
}

[[nodiscard]] auto tryParseSectionLine(std::string_view content) -> std::optional<std::string> {
  if (content.front() != '[' || content.back() != ']') {
    return std::nullopt;
  }
  return std::string{trim(content.substr(1, content.size() - 2U))};
}

[[nodiscard]] auto parseAssignment(std::string_view content)
    -> Result<std::pair<std::string, std::string>> {
  const auto separator = content.find('=');
  if (separator == std::string_view::npos) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Invalid config assignment line: " + std::string{content}});
  }

  const auto key = std::string{trim(content.substr(0, separator))};
  if (key.empty()) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Config key is empty: " + std::string{content}});
  }

  auto rawValue = std::string{trim(content.substr(separator + 1))};
  return std::make_pair(key, rawValue);
}

auto readLineOrError(std::ifstream &file, std::string &line) -> Result<void> {
  if (std::getline(file, line)) {
    return {};
  }
  return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                   .message = "Unterminated array value in config file."});
}

auto extendMultilineArrayValue(std::ifstream &file, std::string &rawValue) -> Result<void> {
  if (trim(rawValue) == "[") {
    for (;;) {
      auto continuation = std::string{};
      if (const auto status = readLineOrError(file, continuation); !status) {
        return tl::make_unexpected(status.error());
      }
      const auto trimmed = trim(stripComment(continuation));
      rawValue += "\n";
      rawValue += std::string{trimmed};
      if (trimmed == "]") {
        return {};
      }
    }
  }

  auto depth = bracketDepthDelta(rawValue);
  while (depth > 0) {
    auto continuation = std::string{};
    if (const auto status = readLineOrError(file, continuation); !status) {
      return tl::make_unexpected(status.error());
    }
    const auto trimmed = trim(stripComment(continuation));
    rawValue += "\n";
    rawValue += std::string{trimmed};
    depth += bracketDepthDelta(trimmed);
  }
  return {};
}

[[nodiscard]] auto parseFloating(std::string_view raw) -> Result<double> {
  const auto value = trim(raw);
  if (value.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Numeric value is empty."});
  }

  auto parsed = 0.0;
  const auto *begin = value.data();
  const auto *end = value.data() + value.size();
  const auto result = std::from_chars(begin, end, parsed);
  if (result.ec != std::errc{} || result.ptr != end || !std::isfinite(parsed)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse floating value."});
  }
  return parsed;
}

} // namespace

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto TextConfig::findRaw(std::string_view section, std::string_view key) const
    -> std::optional<std::string_view> {
  const auto sectionIt = sections_.find(std::string{section});
  if (sectionIt == sections_.end()) {
    return std::nullopt;
  }
  const auto keyIt = sectionIt->second.find(std::string{key});
  if (keyIt == sectionIt->second.end()) {
    return std::nullopt;
  }
  return keyIt->second;
}

auto TextConfig::setValue(std::string_view section, std::string_view key, std::string value)
    -> void {
  sections_[std::string{section}][std::string{key}] = std::move(value);
}

auto loadTextConfig(std::string_view path) -> Result<TextConfig> {
  auto file = std::ifstream{std::string{path}};
  if (!file.is_open()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput,
              .message = "Failed to open config file: " + std::string{path}});
  }

  auto config = TextConfig{};
  auto currentSection = std::string{};

  for (std::string line; std::getline(file, line);) {
    const auto withoutComment = stripComment(line);
    const auto content = trim(withoutComment);
    if (content.empty()) {
      continue;
    }

    if (const auto parsedSection = tryParseSectionLine(content); parsedSection.has_value()) {
      currentSection = *parsedSection;
      continue;
    }

    auto parsedAssignment = parseAssignment(content);
    if (!parsedAssignment) {
      return tl::make_unexpected(parsedAssignment.error());
    }
    auto [key, rawValue] = std::move(*parsedAssignment);

    if (const auto status = extendMultilineArrayValue(file, rawValue); !status) {
      return tl::make_unexpected(status.error());
    }

    config.setValue(currentSection, key, rawValue);
  }

  return config;
}

auto parseQuotedString(std::string_view raw) -> Result<std::string> {
  const auto value = trim(raw);
  if (value.size() < 2U) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "String value must be quoted."});
  }

  const auto first = value.front();
  const auto last = value.back();
  if (first != last || (first != '"' && first != '\'')) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "String value must be quoted."});
  }

  return std::string{value.substr(1, value.size() - 2U)};
}

auto parseDoubleValue(std::string_view raw) -> Result<double> { return parseFloating(raw); }

auto parseIntValue(std::string_view raw) -> Result<int> {
  const auto value = trim(raw);
  if (value.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Integer value is empty."});
  }

  auto parsed = int{0};
  const auto *begin = value.data();
  const auto *end = value.data() + value.size();
  const auto result = std::from_chars(begin, end, parsed);
  if (result.ec != std::errc{} || result.ptr != end) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse integer value."});
  }
  return parsed;
}

auto parseBoolValue(std::string_view raw) -> Result<bool> {
  const auto value = trim(raw);
  if (value == "true") {
    return true;
  }
  if (value == "false") {
    return false;
  }
  return tl::make_unexpected(
      Error{.code = ErrorCode::InvalidInput, .message = "Failed to parse boolean value."});
}

auto parseArrayFlat(std::string_view raw) -> Result<std::vector<double>> {
  const auto value = trim(raw);
  if (value.size() < 2U || value.front() != '[' || value.back() != ']') {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Array value must use [ ... ]."});
  }

  auto numbers = std::vector<double>{};
  auto token = std::string{};
  auto quote = char{0};
  for (const auto character : value) {
    if (quote == 0 && (character == '"' || character == '\'')) {
      quote = character;
      continue;
    }
    if (quote != 0 && character == quote) {
      quote = 0;
      continue;
    }
    if (quote != 0) {
      continue;
    }

    const auto isNumericChar = std::isdigit(static_cast<unsigned char>(character)) != 0 ||
                               character == '-' || character == '+' || character == '.' ||
                               character == 'e' || character == 'E';

    if (isNumericChar) {
      token.push_back(character);
      continue;
    }

    if (!token.empty()) {
      const auto parsed = parseDoubleValue(token);
      if (!parsed) {
        return tl::make_unexpected(parsed.error());
      }
      numbers.push_back(*parsed);
      token.clear();
    }
  }

  if (!token.empty()) {
    const auto parsed = parseDoubleValue(token);
    if (!parsed) {
      return tl::make_unexpected(parsed.error());
    }
    numbers.push_back(*parsed);
  }

  if (numbers.empty()) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Array has no numeric values."});
  }

  return numbers;
}

} // namespace ad::config

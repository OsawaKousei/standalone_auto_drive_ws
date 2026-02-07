#include "map_loader.hpp"

#include <array>
#include <cctype>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace ad {

namespace {

const int kMaxPgmValue = 255;

struct MapYamlConfig {
  const std::string imagePath;
  const double resolution;
  const std::array<double, 3> origin;
  const int negate;
  const double occupiedThreshold;
  const double freeThreshold;
};

struct PgmData {
  const int width;
  const int height;
  const int maxValue;
  const std::vector<unsigned char> pixels;
};

[[nodiscard]] auto trim(std::string_view value) -> std::string_view {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string_view::npos) {
    return {};
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

[[nodiscard]] auto stripQuotes(std::string_view value) -> std::string {
  if (value.size() >= 2U && ((value.front() == '"' && value.back() == '"') ||
                             (value.front() == '\'' && value.back() == '\''))) {
    return std::string{value.substr(1, value.size() - 2U)};
  }
  return std::string{value};
}

[[nodiscard]] auto parseDouble(std::string_view value) -> std::optional<double> {
  try {
    const auto parsed = std::stod(std::string{value});
    return parsed;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

[[nodiscard]] auto parseInt(std::string_view value) -> std::optional<int> {
  try {
    const auto parsed = std::stoi(std::string{value});
    return parsed;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

[[nodiscard]] auto parseOrigin(std::string_view value) -> std::optional<std::array<double, 3>> {
  auto content = trim(value);
  if (content.size() < 2U || content.front() != '[' || content.back() != ']') {
    return std::nullopt;
  }
  content = trim(content.substr(1, content.size() - 2U));
  auto stream = std::istringstream{std::string{content}};
  auto parts = std::vector<double>{};
  while (stream) {
    std::string token;
    if (!std::getline(stream, token, ',')) {
      break;
    }
    const auto parsed = parseDouble(trim(token));
    if (!parsed) {
      return std::nullopt;
    }
    parts.push_back(*parsed);
  }
  if (parts.size() != 3U) {
    return std::nullopt;
  }
  return std::array<double, 3>{parts[0], parts[1], parts[2]};
}

[[nodiscard]] auto readNextToken(std::istream &stream) -> std::optional<std::string> {
  auto token = std::string{};
  for (char ch = 0; stream.get(ch);) {
    if (std::isspace(static_cast<unsigned char>(ch)) != 0) {
      if (!token.empty()) {
        break;
      }
      continue;
    }
    if (ch == '#') {
      std::string ignored;
      std::getline(stream, ignored);
      if (!token.empty()) {
        break;
      }
      continue;
    }
    token.push_back(ch);
  }
  if (token.empty()) {
    return std::nullopt;
  }
  return token;
}

[[nodiscard]] auto loadYamlConfig(std::string_view yamlPath) -> Result<MapYamlConfig> {
  auto file = std::ifstream{std::string{yamlPath}};
  if (!file) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to open map yaml."});
  }

  auto image = std::optional<std::string>{};
  auto resolution = std::optional<double>{};
  auto origin = std::optional<std::array<double, 3>>{};
  auto negate = std::optional<int>{};
  auto occupiedThreshold = std::optional<double>{};
  auto freeThreshold = std::optional<double>{};

  for (std::string line; std::getline(file, line);) {
    auto content = std::string_view{line};
    const auto commentPos = content.find('#');
    if (commentPos != std::string_view::npos) {
      content = content.substr(0, commentPos);
    }
    content = trim(content);
    if (content.empty()) {
      continue;
    }

    const auto separator = content.find(':');
    if (separator == std::string_view::npos) {
      continue;
    }

    const auto key = trim(content.substr(0, separator));
    const auto rawValue = trim(content.substr(separator + 1));

    if (key == "image") {
      image = stripQuotes(rawValue);
    } else if (key == "resolution") {
      resolution = parseDouble(rawValue);
    } else if (key == "origin") {
      origin = parseOrigin(rawValue);
    } else if (key == "negate") {
      negate = parseInt(rawValue);
    } else if (key == "occupied_thresh") {
      occupiedThreshold = parseDouble(rawValue);
    } else if (key == "free_thresh") {
      freeThreshold = parseDouble(rawValue);
    }
  }

  if (!image || !resolution || !origin || !negate || !occupiedThreshold || !freeThreshold) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Map yaml is missing fields."});
  }

  if (*resolution <= 0.0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Map resolution must be positive."});
  }

  if (*negate != 0 && *negate != 1) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Map negate must be 0 or 1."});
  }

  if (*occupiedThreshold < 0.0 || *occupiedThreshold > 1.0 || *freeThreshold < 0.0 ||
      *freeThreshold > 1.0 || *freeThreshold >= *occupiedThreshold) {
    return tl::make_unexpected(Error{.code = ErrorCode::InvalidInput,
                                     .message = "Map thresholds are invalid or overlapping."});
  }

  return MapYamlConfig{.imagePath = *image,
                       .resolution = *resolution,
                       .origin = *origin,
                       .negate = *negate,
                       .occupiedThreshold = *occupiedThreshold,
                       .freeThreshold = *freeThreshold};
}

[[nodiscard]] auto loadPgm(std::string_view pgmPath) -> Result<PgmData> {
  auto stream = std::ifstream{std::string{pgmPath}, std::ios::binary};
  if (!stream) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "Failed to open PGM image."});
  }

  const auto magic = readNextToken(stream);
  if (!magic || *magic != "P5") {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "PGM header is not P5."});
  }

  const auto widthToken = readNextToken(stream);
  const auto heightToken = readNextToken(stream);
  const auto maxToken = readNextToken(stream);
  if (!widthToken || !heightToken || !maxToken) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "PGM header is incomplete."});
  }

  const auto width = parseInt(*widthToken);
  const auto height = parseInt(*heightToken);
  const auto maxValue = parseInt(*maxToken);
  if (!width || !height || !maxValue || *width <= 0 || *height <= 0 || *maxValue <= 0) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "PGM header has invalid values."});
  }

  if (*maxValue > kMaxPgmValue) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "PGM max value must be <= 255."});
  }

  stream >> std::ws;
  const auto pixelCount = static_cast<std::size_t>(*width) * static_cast<std::size_t>(*height);
  auto raw = std::string(pixelCount, '\0');
  stream.read(raw.data(), static_cast<std::streamsize>(pixelCount));
  if (stream.gcount() != static_cast<std::streamsize>(pixelCount)) {
    return tl::make_unexpected(
        Error{.code = ErrorCode::InvalidInput, .message = "PGM pixel data is incomplete."});
  }

  auto pixels = std::vector<unsigned char>{};
  pixels.reserve(pixelCount);
  for (const auto rawByte : raw) {
    pixels.push_back(static_cast<unsigned char>(rawByte));
  }

  return PgmData{
      .width = *width, .height = *height, .maxValue = *maxValue, .pixels = std::move(pixels)};
}

[[nodiscard]] auto toOccupancy(const MapYamlConfig &config, int pixelValue, int maxValue) -> int {
  const auto normalized = static_cast<double>(pixelValue) / static_cast<double>(maxValue);
  const auto occupancy = (config.negate == 0) ? (1.0 - normalized) : normalized;
  if (occupancy > config.occupiedThreshold) {
    return 1;
  }
  if (occupancy < config.freeThreshold) {
    return 0;
  }
  return 1;
}

} // namespace

auto loadMapFromYaml(std::string_view yamlPath) -> Result<types::MapData> {
  const auto configResult = loadYamlConfig(yamlPath);
  if (!configResult) {
    return tl::make_unexpected(configResult.error());
  }

  const auto yamlPathFs = std::filesystem::path{std::string{yamlPath}};
  const auto imagePath = [&]() -> std::filesystem::path {
    const auto image = std::filesystem::path{configResult->imagePath};
    return image.is_absolute() ? image : (yamlPathFs.parent_path() / image);
  }();

  const auto pgmResult = loadPgm(imagePath.string());
  if (!pgmResult) {
    return tl::make_unexpected(pgmResult.error());
  }

  const auto width = pgmResult->width;
  const auto height = pgmResult->height;
  const auto resolution = configResult->resolution;

  const auto grid = [&]() -> std::vector<std::int8_t> {
    const auto widthSize = static_cast<std::size_t>(width);
    const auto heightSize = static_cast<std::size_t>(height);
    auto data = std::vector<std::int8_t>(widthSize * heightSize, 0);
    for (const auto row : std::views::iota(std::size_t{0}, heightSize)) {
      const auto sourceRow = (heightSize - 1U) - row;
      for (const auto col : std::views::iota(std::size_t{0}, widthSize)) {
        const auto sourceIndex = (sourceRow * widthSize) + col;
        const auto targetIndex = (row * widthSize) + col;
        const auto pixelValue = static_cast<int>(pgmResult->pixels[sourceIndex]);
        data[targetIndex] =
            static_cast<std::int8_t>(toOccupancy(*configResult, pixelValue, pgmResult->maxValue));
      }
    }
    return data;
  }();

  return types::MapData{
      .width = width, .height = height, .resolution = resolution, .grid = std::move(grid)};
}

} // namespace ad

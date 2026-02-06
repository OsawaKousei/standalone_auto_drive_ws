#include "greeting_service.hpp"
#include <fmt/core.h>

namespace features::greeting {
auto GreetingService::createMessage(const GreetingData &data) const -> Result<std::string> {
  // ガード節による早期リターン
  if (data.name.empty()) {
    return tl::make_unexpected(ErrorCode::InvalidInput);
  }

  // fmt を使用した型安全なフォーマット
  return fmt::format("Hello, {}! Welcome to Modern C++ world.", data.name);
}
} // namespace features::greeting
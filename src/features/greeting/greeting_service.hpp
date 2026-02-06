#pragma once
#include "../../shared/result.hpp"
#include "greeting_data.hpp"

namespace features::greeting {
class GreetingService {
public:
  // 状態を変更しない const メソッド
  [[nodiscard]] auto createMessage(const GreetingData &data) const -> Result<std::string>;
};
} // namespace features::greeting
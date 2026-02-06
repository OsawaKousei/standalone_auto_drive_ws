#include "features/greeting/greeting_service.hpp"
#include <fmt/core.h>
#include <ranges>
#include <vector>

auto main() -> int {
  using namespace features::greeting;

  // C++20 Ranges を使用した宣言的なデータ処理
  const std::vector<GreetingData> users = {{"Alice"}, {""}, {"Bob"}};

  const auto service = GreetingService{};

  auto results =
      users | std::views::transform([&](const auto &user) -> decltype(service.createMessage(user)) {
        return service.createMessage(user);
      });

  for (const auto &res : results) {
    if (res) {
      fmt::print("{}\n", *res); // fmt::print による出力
    } else {
      fmt::print(stderr, "Error: Invalid user data provided.\n");
    }
  }

  return 0;
}
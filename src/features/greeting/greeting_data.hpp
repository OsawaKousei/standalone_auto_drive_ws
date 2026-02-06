#pragma once
#include <string>

namespace features::greeting {
struct GreetingData {
  const std::string name; // Immutable by default
};
} // namespace features::greeting
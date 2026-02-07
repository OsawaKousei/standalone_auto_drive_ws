#pragma once

#include <array>

namespace ad::math {

using Matrix3 = std::array<double, 9>;

[[nodiscard]] auto multiply(const Matrix3 &left, const Matrix3 &right) -> Matrix3;
[[nodiscard]] auto transpose(const Matrix3 &matrix) -> Matrix3;

} // namespace ad::math

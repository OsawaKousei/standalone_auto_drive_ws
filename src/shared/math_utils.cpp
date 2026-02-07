#include "math_utils.hpp"

namespace ad::math {

auto multiply(const Matrix3 &left, const Matrix3 &right) -> Matrix3 {
  auto result = Matrix3{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += left[static_cast<std::size_t>(row * 3 + k)] *
               right[static_cast<std::size_t>(k * 3 + col)];
      }
      result[static_cast<std::size_t>(row * 3 + col)] = sum;
    }
  }
  return result;
}

auto transpose(const Matrix3 &matrix) -> Matrix3 {
  return {matrix[0], matrix[3], matrix[6], matrix[1], matrix[4],
          matrix[7], matrix[2], matrix[5], matrix[8]};
}

} // namespace ad::math

#pragma once

#include <string>
#include <tl/expected.hpp>

namespace ad {

// 予測可能なエラーを明示するためのコード種別
enum class ErrorCode { InvalidInput, EmptyCollection, SizeMismatch };

// 失敗理由を伝搬するための不変データ
struct Error {
  const ErrorCode code;
  const std::string message;
};

template <typename T> using Result = tl::expected<T, Error>;
using Status = tl::expected<void, Error>;

} // namespace ad
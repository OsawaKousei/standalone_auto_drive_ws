#pragma once
#include <string>
#include <tl/expected.hpp>

// エラー型を定義
enum class ErrorCode { InvalidInput, SystemError };

// Result型のエイリアス設定
template <typename T> using Result = tl::expected<T, ErrorCode>;
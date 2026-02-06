cmake_minimum_required(VERSION 3.22)
project(CppTemplate LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- 依存関係の定義 ---
include(FetchContent)

# 1. tl::expected
set(EXPECTED_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  expected
  GIT_REPOSITORY https://github.com/TartanLlama/expected.git
  GIT_TAG        v1.3.1
)
FetchContent_MakeAvailable(expected)

# 2. fmt
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        12.1.0
)
FetchContent_MakeAvailable(fmt)

# --- ビルド設定 ---
add_executable(app 
    src/main.cpp 
    src/features/greeting/greeting_service.cpp
)

# Sanitizer のオプション化
option(ENABLE_SANITIZER "Enable address and undefined sanitizer" ON)

target_compile_options(app PRIVATE 
    -Wall -Wextra -Werror 
    -fno-omit-frame-pointer
)

if(ENABLE_SANITIZER)
    target_compile_options(app PRIVATE -fsanitize=address,undefined)
    target_link_options(app PRIVATE -fsanitize=address,undefined)
endif()

target_link_libraries(app PRIVATE fmt::fmt tl::expected)
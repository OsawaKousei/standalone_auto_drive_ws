cmake_minimum_required(VERSION 3.22)
project(AutonomousDrivingModern LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(COMMON_COMPILE_OPTIONS
  -Wall
  -Wextra
  -Werror
  -fno-omit-frame-pointer)

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

# 3. GoogleTest
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2
)
FetchContent_MakeAvailable(googletest)

enable_testing()

# --- ビルド設定 ---
add_library(ad_core
  src/features/planning/astar_planner.cpp
  src/features/control/pure_pursuit.cpp
  src/features/localization/ransac_localizer.cpp
  src/features/simulation/unicycle_model.cpp
  src/features/simulation/lidar_sim.cpp
  src/features/visualization/visualizer.cpp
)

# Sanitizer のオプション化
option(ENABLE_SANITIZER "Enable address and undefined sanitizer" ON)

target_compile_options(ad_core PRIVATE ${COMMON_COMPILE_OPTIONS})

if(ENABLE_SANITIZER)
    target_compile_options(ad_core PRIVATE -fsanitize=address,undefined)
    target_link_options(ad_core PRIVATE -fsanitize=address,undefined)
endif()

target_link_libraries(ad_core PUBLIC fmt::fmt tl::expected)
target_include_directories(ad_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(app src/main.cpp)
target_link_libraries(app PRIVATE ad_core)
target_compile_options(app PRIVATE ${COMMON_COMPILE_OPTIONS})
if(ENABLE_SANITIZER)
    target_compile_options(app PRIVATE -fsanitize=address,undefined)
    target_link_options(app PRIVATE -fsanitize=address,undefined)
endif()

add_executable(simulation_tests
  test/simulation_visualization_test.cpp
)
target_link_libraries(simulation_tests PRIVATE ad_core GTest::gtest_main)
target_compile_options(simulation_tests PRIVATE ${COMMON_COMPILE_OPTIONS})
if(ENABLE_SANITIZER)
    target_compile_options(simulation_tests PRIVATE -fsanitize=address,undefined)
    target_link_options(simulation_tests PRIVATE -fsanitize=address,undefined)
endif()

include(GoogleTest)
gtest_discover_tests(simulation_tests)
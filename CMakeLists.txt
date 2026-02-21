cmake_minimum_required(VERSION 3.22)
project(AutonomousDrivingModern LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- 共通コンパイルオプションの設定 ---
add_library(project_options INTERFACE)
target_compile_options(project_options INTERFACE
  -Wall -Wextra -Werror -fno-omit-frame-pointer -march=native
)

if(ENABLE_SANITIZER)
    target_compile_options(project_options INTERFACE -fsanitize=address,undefined,leak)
    target_link_options(project_options INTERFACE -fsanitize=address,undefined,leak)
endif()

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

# 4. Eigen (header-only)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

# 5. matplotlib-cpp (header-only)
FetchContent_Declare(
  matplotlibcpp
  GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(matplotlibcpp)

# --- Python / 依存パッケージ の検出 ---
if(DEFINED ENV{VIRTUAL_ENV})
  set(Python3_ROOT_DIR "$ENV{VIRTUAL_ENV}")
  # システムのPythonを混ぜないよう、仮想環境内のみを探す設定
  set(Python3_FIND_VIRTUALENV ONLY)
endif()

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# --- ライブラリ定義 ---
add_library(ad_core
  src/features/planning/astar_planner.cpp
  src/features/planning/dijkstra_planner.cpp
  src/features/planning/grid_collision_checker.cpp
  src/features/planning/planner_utils.cpp
  src/features/control/pure_pursuit.cpp
  src/features/localization/ekf_localizer.cpp
  src/features/localization/localizer_util.cpp
  src/features/simulation/collision_checker.cpp
  src/features/simulation/unicycle_model.cpp
  src/features/simulation/lidar_sim.cpp
  src/features/visualization/visualizer.cpp
  src/shared/map_loader.cpp
)

target_link_libraries(ad_core 
  PUBLIC 
    project_options
    fmt::fmt 
    tl::expected 
    Eigen3::Eigen 
    Python3::Python 
    Python3::NumPy
)

# matplotlib-cpp はヘッダーのみなので include パスを通す
target_include_directories(ad_core PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${matplotlibcpp_SOURCE_DIR}
)

# --- 実行ファイル追加用のマクロ (コードの重複を排除) ---
macro(add_ad_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ad_core project_options)
endmacro()

add_ad_executable(auto_drive_app src/auto_drive_app.cpp)
add_ad_executable(auto_drive_log_replay src/auto_drive_log_replay.cpp)
add_ad_executable(visualization_demo test/visualization_demo.cpp)
add_ad_executable(planning_visualization_demo test/planning_visualization_demo.cpp)
add_ad_executable(planning_control_demo test/planning_control_demo.cpp)
add_ad_executable(planning_control_lidar_demo test/planning_control_lidar_demo.cpp)
add_ad_executable(localization_control_lidar_demo test/localization_control_lidar_demo.cpp)

# テストの設定
enable_testing()
add_executable(planning_test test/planning_test.cpp)
target_link_libraries(planning_test PRIVATE ad_core project_options GTest::gtest_main)

add_executable(simulation_tests test/simulation_visualization_test.cpp)
target_link_libraries(simulation_tests PRIVATE ad_core project_options GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(planning_test)
gtest_discover_tests(simulation_tests)
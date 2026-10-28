cmake_minimum_required(VERSION 3.22)
project(AutonomousDrivingModern LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

set(AD_VISUALIZATION_BACKEND "wgpu" CACHE STRING "Rendering backend: wgpu or matplotlib")
set_property(CACHE AD_VISUALIZATION_BACKEND PROPERTY STRINGS wgpu matplotlib)
if(NOT AD_VISUALIZATION_BACKEND MATCHES "^(wgpu|matplotlib)$")
  message(FATAL_ERROR "AD_VISUALIZATION_BACKEND must be wgpu or matplotlib")
endif()
message(STATUS "Visualization backend: ${AD_VISUALIZATION_BACKEND}")
include(CTest)

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
# Register this project's tests without adding Eigen's own test suite.
set(_ad_build_testing "${BUILD_TESTING}")
set(BUILD_TESTING OFF)
FetchContent_MakeAvailable(eigen)
set(BUILD_TESTING "${_ad_build_testing}")

if(AD_VISUALIZATION_BACKEND STREQUAL "wgpu")
  include(cmake/RustVisualization.cmake)
  set(_ad_visualization_source src/features/visualization/visualizer.cpp)
else()
  FetchContent_Declare(
    matplotlibcpp
    GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(matplotlibcpp)
  if(DEFINED ENV{VIRTUAL_ENV})
    set(Python3_ROOT_DIR "$ENV{VIRTUAL_ENV}")
    set(Python3_FIND_VIRTUALENV FIRST)
  endif()
  find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
  set(_ad_visualization_source src/features/visualization/visualizer_matplotlib.cpp)
endif()

# --- ライブラリ定義 ---
add_library(ad_core
  src/features/planning/planner/astar_planner.cpp
  src/features/planning/planner/dijkstra_planner.cpp
  src/features/planning/planner_factory.cpp
  src/features/planning/collision_checker/grid_collision_checker.cpp
  src/features/planning/planner_utils.cpp
  src/features/control/controller/pure_pursuit.cpp
  src/features/control/controller/pid.cpp
  src/features/control/controller_factory.cpp
  src/features/localization/localizer/ekf_localizer.cpp
  src/features/localization/observation_model/line_extractor.cpp
  src/features/localization/observation_model/simple_line_association_model.cpp
  src/features/localization/observation_model/ransac_line_association_model.cpp
  src/features/localization/observation_model/line_observation_util.cpp
  src/features/localization/localizer_factory.cpp
  src/features/localization/localizer_util.cpp
  src/features/simulation/collision_checker/collision_checker.cpp
  src/features/simulation/physics/unicycle_model.cpp
  src/features/simulation/physics/holonomic_model.cpp
  src/features/simulation/sensor/lidar_sensor.cpp
  src/features/simulation/sensor/odometry_sensor.cpp
  src/features/simulation/simulation_factory.cpp
  ${_ad_visualization_source}
  src/shared/map_loader.cpp
  src/shared/text_config.cpp
  src/shared/scenario_runtime.cpp
)

target_link_libraries(ad_core 
  PUBLIC 
    project_options
    fmt::fmt 
    tl::expected 
    Eigen3::Eigen 
)

target_include_directories(ad_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
if(AD_VISUALIZATION_BACKEND STREQUAL "wgpu")
  target_link_libraries(ad_core PRIVATE ad_rust_visualization)
else()
  target_link_libraries(ad_core PRIVATE Python3::Python Python3::NumPy)
  target_include_directories(ad_core SYSTEM PRIVATE ${matplotlibcpp_SOURCE_DIR})
endif()

# --- 実行ファイル追加用のマクロ (コードの重複を排除) ---
macro(add_ad_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ad_core project_options)
endmacro()

add_ad_executable(auto_drive_app src/auto_drive_app.cpp)
add_ad_executable(auto_drive_log_replay src/auto_drive_log_replay.cpp)
add_ad_executable(control_test_app test/control/control_test_app.cpp)
add_ad_executable(planning_test_app test/planning/planning_test_app.cpp)
add_ad_executable(localization_test_app test/localization/localization_test_app.cpp)
add_ad_executable(line_extractor_overlay_test_app test/localization/line_extractor_overlay_test_app.cpp)
add_ad_executable(observation_model_test_app test/localization/observation_model_test_app.cpp)

if(BUILD_TESTING AND AD_VISUALIZATION_BACKEND STREQUAL "wgpu")
  add_ad_executable(visualizer_test test/visualization/visualizer_test.cpp)
  target_link_libraries(visualizer_test PRIVATE GTest::gtest_main)
  add_test(NAME visualizer_cpu COMMAND visualizer_test --gtest_filter=-VisualizerOutput.*)
  # These integration tests require an available Vulkan adapter. Window additionally needs X11.
  option(AD_TEST_RENDER_OUTPUT "Enable Vulkan PNG integration tests" OFF)
  option(AD_TEST_RENDER_WINDOW "Enable X11/Vulkan window integration tests" OFF)
  if(AD_TEST_RENDER_OUTPUT)
    add_test(NAME visualizer_png COMMAND visualizer_test --gtest_filter=VisualizerOutput.Png)
    set_tests_properties(visualizer_png PROPERTIES ENVIRONMENT "DISPLAY=;WAYLAND_DISPLAY=")
  endif()
  if(AD_TEST_RENDER_WINDOW)
    add_test(NAME visualizer_window COMMAND visualizer_test --gtest_filter=VisualizerOutput.Window)
  endif()
endif()

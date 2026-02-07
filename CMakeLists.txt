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

# 4. matplotlib-cpp (header-only)
FetchContent_Declare(
  matplotlibcpp
  GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
  GIT_TAG        master
)
FetchContent_MakeAvailable(matplotlibcpp)

if(DEFINED ENV{VIRTUAL_ENV})
  set(Python3_ROOT_DIR "$ENV{VIRTUAL_ENV}")
  set(Python3_FIND_VIRTUALENV FIRST)
endif()

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

enable_testing()

# --- ビルド設定 ---
add_library(ad_core
  src/features/planning/astar_planner.cpp
  src/features/planning/dijkstra_planner.cpp
  src/features/planning/planner_utils.cpp
  src/features/control/pure_pursuit.cpp
  src/features/localization/ransac_localizer.cpp
  src/features/simulation/unicycle_model.cpp
  src/features/simulation/lidar_sim.cpp
  src/features/visualization/visualizer.cpp
  src/shared/map_loader.cpp
)


target_compile_options(ad_core PRIVATE ${COMMON_COMPILE_OPTIONS})

if(ENABLE_SANITIZER)
    target_compile_options(ad_core PRIVATE -fsanitize=address,undefined,leak)
    target_link_options(ad_core PRIVATE -fsanitize=address,undefined,leak)
endif()

target_link_libraries(ad_core PUBLIC fmt::fmt tl::expected)
target_include_directories(ad_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(ad_core PUBLIC Python3::Python)
target_include_directories(ad_core PUBLIC ${matplotlibcpp_SOURCE_DIR})

if(NOT Python3_NumPy_INCLUDE_DIRS)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE Python3_NumPy_INCLUDE_DIRS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_VARIABLE _numpy_err
    RESULT_VARIABLE _numpy_status
  )
  if(NOT _numpy_status EQUAL 0 OR Python3_NumPy_INCLUDE_DIRS STREQUAL "")
    message(FATAL_ERROR "NumPy headers not found. Ensure numpy is installed in the active Python (e.g., uv pip install numpy).
Error: ${_numpy_err}")
  endif()
endif()

target_include_directories(ad_core PUBLIC ${Python3_NumPy_INCLUDE_DIRS})

add_executable(app src/main.cpp)
target_link_libraries(app PRIVATE ad_core)
target_compile_options(app PRIVATE ${COMMON_COMPILE_OPTIONS})
if(ENABLE_SANITIZER)
    target_compile_options(app PRIVATE -fsanitize=address,undefined,leak)
    target_link_options(app PRIVATE -fsanitize=address,undefined,leak)
endif()

add_executable(simulation_tests
  test/simulation_visualization_test.cpp
)
target_link_libraries(simulation_tests PRIVATE ad_core GTest::gtest_main)
target_compile_options(simulation_tests PRIVATE ${COMMON_COMPILE_OPTIONS})
if(ENABLE_SANITIZER)
    target_compile_options(simulation_tests PRIVATE -fsanitize=address,undefined,leak)
    target_link_options(simulation_tests PRIVATE -fsanitize=address,undefined,leak)
endif()

add_executable(planning_tests
  test/planning_test.cpp
)
target_link_libraries(planning_tests PRIVATE ad_core GTest::gtest_main)
target_compile_options(planning_tests PRIVATE ${COMMON_COMPILE_OPTIONS})
if(ENABLE_SANITIZER)
    target_compile_options(planning_tests PRIVATE -fsanitize=address,undefined,leak)
    target_link_options(planning_tests PRIVATE -fsanitize=address,undefined,leak)
endif()

add_executable(visualization_demo
  test/visualization_demo.cpp
)
target_link_libraries(visualization_demo PRIVATE ad_core fmt::fmt)
target_compile_options(visualization_demo PRIVATE ${COMMON_COMPILE_OPTIONS})
if(ENABLE_SANITIZER)
  target_compile_options(visualization_demo PRIVATE -fsanitize=address,undefined,leak)
  target_link_options(visualization_demo PRIVATE -fsanitize=address,undefined,leak)
endif()

add_executable(planning_visualization_demo
  test/planning_visualization_demo.cpp
)
target_link_libraries(planning_visualization_demo PRIVATE ad_core fmt::fmt)
target_compile_options(planning_visualization_demo PRIVATE ${COMMON_COMPILE_OPTIONS})
if(ENABLE_SANITIZER)
  target_compile_options(planning_visualization_demo PRIVATE -fsanitize=address,undefined,leak)
  target_link_options(planning_visualization_demo PRIVATE -fsanitize=address,undefined,leak)
endif()

include(GoogleTest)
gtest_discover_tests(simulation_tests)
gtest_discover_tests(planning_tests)
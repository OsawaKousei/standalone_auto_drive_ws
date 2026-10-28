# Build the existing C ABI without changing the reference Rust sources.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The wgpu backend currently requires Linux. Use AD_VISUALIZATION_BACKEND=matplotlib on other platforms.")
endif()
set(AD_RUST_SOURCE_DIR "${PROJECT_SOURCE_DIR}/ref/rust-wgpu-plotting" CACHE PATH
    "Path to the rust-wgpu-plotting workspace")
if(NOT EXISTS "${AD_RUST_SOURCE_DIR}/crates/ad-visualization-ffi/include/ad_visualization_ffi.h")
  message(FATAL_ERROR "rust-wgpu-plotting is missing. Set AD_RUST_SOURCE_DIR to its workspace.")
endif()
find_program(AD_CARGO_EXECUTABLE cargo REQUIRED)
set(_ad_rust_target_dir "${CMAKE_BINARY_DIR}/rust-visualization")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(_ad_cargo_profile dev)
  set(_ad_rust_profile_dir debug)
else()
  set(_ad_cargo_profile release)
  set(_ad_rust_profile_dir release)
endif()
set(_ad_rust_library "${_ad_rust_target_dir}/${_ad_rust_profile_dir}/libad_visualization_ffi.so")
file(GLOB_RECURSE _ad_rust_inputs CONFIGURE_DEPENDS
  "${AD_RUST_SOURCE_DIR}/crates/*.rs"
  "${AD_RUST_SOURCE_DIR}/crates/*/Cargo.toml"
  "${AD_RUST_SOURCE_DIR}/crates/*.wgsl"
  "${AD_RUST_SOURCE_DIR}/crates/*.ttf")
add_custom_command(
  OUTPUT "${_ad_rust_library}"
  COMMAND "${CMAKE_COMMAND}" -E env "CARGO_TARGET_DIR=${_ad_rust_target_dir}"
          "${AD_CARGO_EXECUTABLE}" build --locked --profile "${_ad_cargo_profile}"
          --manifest-path "${AD_RUST_SOURCE_DIR}/Cargo.toml"
          -p ad-visualization-ffi --features native
  DEPENDS ${_ad_rust_inputs} "${AD_RUST_SOURCE_DIR}/Cargo.toml"
          "${AD_RUST_SOURCE_DIR}/Cargo.lock"
  COMMENT "Building the Rust wgpu visualization backend"
  VERBATIM)
add_custom_target(ad_rust_visualization_build DEPENDS "${_ad_rust_library}")
add_library(ad_rust_visualization SHARED IMPORTED GLOBAL)
set_target_properties(ad_rust_visualization PROPERTIES
  IMPORTED_LOCATION "${_ad_rust_library}"
  IMPORTED_NO_SONAME TRUE
  INTERFACE_INCLUDE_DIRECTORIES "${AD_RUST_SOURCE_DIR}/crates/ad-visualization-ffi/include")
add_dependencies(ad_rust_visualization ad_rust_visualization_build)

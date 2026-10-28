# Consume the pinned, prebuilt C ABI release; no Rust sources/toolchain are needed.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR
   NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  message(FATAL_ERROR "The wgpu release requires Linux x86_64. Use AD_VISUALIZATION_BACKEND=matplotlib on other platforms.")
endif()

set(AD_VISUALIZATION_ROOT "" CACHE PATH "Path to an extracted ad-visualization v0.1.0 Linux x86_64 bundle (empty: download release)")
if(AD_VISUALIZATION_ROOT)
  set(_ad_visualization_root "${AD_VISUALIZATION_ROOT}")
else()
  FetchContent_Declare(ad_visualization_bundle
    URL https://github.com/OsawaKousei/rust-wgpu-plotting/releases/download/v0.1.0/ad-visualization-v0.1.0-linux-x86_64.tar.gz
    URL_HASH SHA256=c0ee20a3d46fb4f98a3f0d794e5ea7bd662c2dae9a7af76626930c641b462436
    TLS_VERIFY TRUE)
  FetchContent_GetProperties(ad_visualization_bundle)
  if(NOT ad_visualization_bundle_POPULATED)
    FetchContent_Populate(ad_visualization_bundle)
  endif()
  set(_ad_visualization_root "${ad_visualization_bundle_SOURCE_DIR}")
endif()

foreach(_ad_file include/ad_visualization_ffi.h lib/libad_visualization_ffi.so)
  if(NOT EXISTS "${_ad_visualization_root}/${_ad_file}")
    message(FATAL_ERROR "Invalid ad-visualization bundle: missing ${_ad_visualization_root}/${_ad_file}")
  endif()
endforeach()
message(STATUS "Visualization release: v0.1.0 (${_ad_visualization_root})")
add_library(ad_rust_visualization SHARED IMPORTED GLOBAL)
# The release library has no SONAME. Avoid recording an absolute build path in
# DT_NEEDED; CMAKE_BUILD_RPATH_USE_ORIGIN keeps the downloaded bundle relocatable.
set_target_properties(ad_rust_visualization PROPERTIES
  IMPORTED_LOCATION "${_ad_visualization_root}/lib/libad_visualization_ffi.so"
  IMPORTED_NO_SONAME TRUE
  INTERFACE_INCLUDE_DIRECTORIES "${_ad_visualization_root}/include")

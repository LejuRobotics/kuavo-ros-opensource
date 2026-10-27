# Fixed header-only ProxSuite 0.7.2 used by the velocity IK solver.
# No configure-time download, submodule setup, installation or system discovery.
find_package(Eigen3 3.0.5 REQUIRED)
add_library(kuavo_proxsuite INTERFACE)
target_include_directories(kuavo_proxsuite SYSTEM INTERFACE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(kuavo_proxsuite INTERFACE Eigen3::Eigen)
target_compile_features(kuavo_proxsuite INTERFACE cxx_std_17)

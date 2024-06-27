cmake_minimum_required(VERSION 3.8)
project(static_pointcloud_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)

# Add submodule
set(LIBLAS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/liblas)
add_subdirectory(${LIBLAS_DIR} ${CMAKE_BINARY_DIR}/liblas)
include_directories(
  include
  ${LIBLAS_DIR}/include
)

add_executable(static_pointcloud_publisher src/static_pointcloud_publisher.cpp)
target_link_libraries(static_pointcloud_publisher las)
ament_target_dependencies(static_pointcloud_publisher rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  static_pointcloud_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()

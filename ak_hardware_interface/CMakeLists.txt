cmake_minimum_required(VERSION 3.8)
project(ak_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

install(
  DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

##COMPILE
add_library(
  ak_hardware_interface
  SHARED
  src/epoll_event_loop.cpp
  src/socket_can.cpp
  src/ak_hardware_interface.cpp
)
target_compile_features(ak_hardware_interface PUBLIC cxx_std_20)
target_include_directories(ak_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ak_hardware_interface>
)
ament_target_dependencies(
  ak_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# prevent pluginlib from using boost
target_compile_definitions(ak_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ak_hardware_interface.xml)

install(TARGETS ak_hardware_interface
  EXPORT export_ak_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/ak_hardware_interface
)

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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_ak_hardware_interface test/test_ak_hardware_interface.cpp)
  target_include_directories(test_ak_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_ak_hardware_interface
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ak_hardware_interface
)

## EXPORTS
ament_export_targets(export_ak_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

cmake_minimum_required(VERSION 3.8)
project(f1tenth_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/car_pose_tools.cpp)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rviz_common
  rviz_default_plugins)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY icons
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  pluginlib
  rviz_common
  rviz_default_plugins)

ament_package()

cmake_minimum_required(VERSION 3.3)
project(rviz_tools2)
# Source: https://github.com/marip8/rviz_tool_cursor/blob/master/CMakeLists.txt

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(dep_pkgs
  rviz2
  pluginlib
  std_msgs
  rviz_common
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
foreach (pkg ${dep_pkgs})
    find_package(${pkg} REQUIRED)
endforeach()

find_package(Qt5 REQUIRED Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
  #src/bool_tool.hpp
  src/auto_start.hpp
  src/auto_stop.hpp
)

add_library(${PROJECT_NAME} SHARED
  #src/bool_tool.cpp
  src/auto_start.cpp
  src/auto_stop.cpp
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_target_dependencies(${PROJECT_NAME}
  ${dep_pkgs}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(
   DIRECTORY icons
   DESTINATION "share/${PROJECT_NAME}"
)

install(
   DIRECTORY plugins
   DESTINATION "share/${PROJECT_NAME}"
)

cmake_minimum_required(VERSION 3.5)
project(rviz_camera_stream)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-error=deprecated-declarations)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#dependencies
find_package(Boost REQUIRED thread)
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

qt5_wrap_cpp(${PROJECT_NAME}_mocs include/${PROJECT_NAME}/camera_display.hpp)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/video_publisher.cpp
  src/camera_display.cpp
  ${${PROJECT_NAME}_mocs}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

# target_link_libraries(${PROJECT_NAME} PUBLIC

# )

ament_target_dependencies(${PROJECT_NAME}
  Boost
  rclcpp
  tf2
  tf2_ros
  image_transport
  sensor_msgs
  std_srvs
  rviz_rendering
  rviz_ogre_vendor
  rviz_common
  rviz_default_plugins
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(
  Boost
  rclcpp
  tf2
  tf2_ros
  image_transport
  sensor_msgs
  std_srvs
  rviz_rendering
  rviz_ogre_vendor
  rviz_common
  rviz_default_plugins
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

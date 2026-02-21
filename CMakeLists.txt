cmake_minimum_required(VERSION 3.5)

project(img_proc)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_tools REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/brightness.cpp
  src/color_tracker.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  geometry_msgs
  image_tools
  vision_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "imgproc::Brightness"
  EXECUTABLE brightness
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "imgproc::ColorTracker"
  EXECUTABLE color_tracker
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  image_tools
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(metal_detection_marker)

cmake_policy(SET CMP0074 NEW)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -g -ggdb -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(metal_detection src/metal_detection.cpp)
target_compile_features(metal_detection PUBLIC c_std_99 cxx_std_17)

target_link_libraries(metal_detection PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)


install(TARGETS metal_detection
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


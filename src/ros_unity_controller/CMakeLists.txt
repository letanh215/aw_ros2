cmake_minimum_required(VERSION 3.8)
project(ros_unity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)

add_executable(simple_converter src/simple_control_converter.cpp)
ament_target_dependencies(simple_converter rclcpp autoware_auto_control_msgs geometry_msgs sensor_msgs autoware_auto_vehicle_msgs std_msgs)

add_executable(odom_update src/odom_update.cpp)
ament_target_dependencies(odom_update rclcpp geometry_msgs tf2_ros)

add_executable(odom_pub src/odom_pub.cpp)
ament_target_dependencies(odom_pub rclcpp geometry_msgs nav_msgs)

install(TARGETS
  simple_converter
  odom_update
  odom_pub
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

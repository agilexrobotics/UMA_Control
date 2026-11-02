cmake_minimum_required(VERSION 3.8)
project(uma_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(uma_sdk REQUIRED)
find_package(uma_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(uma_ros_node src/uma_ros_node.cpp src/uma_ros_messenger.cpp)
target_include_directories(uma_ros_node PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(uma_ros_node uma_sdk::uma_sdk Eigen3::Eigen)
ament_target_dependencies(uma_ros_node rclcpp rcl_interfaces std_msgs sensor_msgs geometry_msgs uma_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)


install(TARGETS
  uma_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/uma_ros_py_node.py
  scripts/recording_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)


ament_package()

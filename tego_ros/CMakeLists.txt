cmake_minimum_required(VERSION 3.8)
project(tego_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tego_sdk REQUIRED)
find_package(tego_msgs REQUIRED)
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

add_executable(tego_ros_node src/tego_ros_node.cpp src/tego_ros_messenger.cpp)
target_include_directories(tego_ros_node PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(tego_ros_node tego_sdk::tego_sdk Eigen3::Eigen)
ament_target_dependencies(tego_ros_node rclcpp rcl_interfaces std_msgs sensor_msgs geometry_msgs tego_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)


install(TARGETS 
  tego_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)


ament_package()

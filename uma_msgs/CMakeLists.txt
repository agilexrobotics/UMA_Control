cmake_minimum_required(VERSION 3.8)
project(uma_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/HandleCalibrationStatus.msg"
  "msg/HandleState.msg"
  "msg/MtSensorStatus.msg"
  "msg/SensorData.msg"

)
set(SRV_FILES
  "srv/ChangeEncoderId.srv"
  "srv/ResetControl.srv"
  "srv/SetBaudRate.srv"
  "srv/SetEncoderMode.srv"
  "srv/SetEncoderReturnRate.srv"
  "srv/SetEncoderZero.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
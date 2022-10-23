cmake_minimum_required(VERSION 3.5)
project(moving_objects)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ALLOW_DUPLICATE_CUSTOM_TARGETS TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(aboat_interfaces REQUIRED)
find_package(position_interfaces REQUIRED)
#find_package(builtin_interfaces REQUIRED)

include_directories(include
${aboat_interfaces_INCLUDE_DIR}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MovingObject.msg"
  "msg/MovingObjectArray.msg"
  "msg/Gpsx.msg"
  "msg/Minimu9AHRS.msg"
  DEPENDENCIES std_msgs 
  #DEPENDENCIES builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(find_moving_objects src/laserscan_interpreter.cpp src/bank.cpp include/bank.h include/LaserScanInterpreter.h)


ament_target_dependencies(find_moving_objects
  rclcpp 
  sensor_msgs 
  geometry_msgs
  std_msgs
  #aboat_interfaces
  #position_interfaces
)
rosidl_target_interfaces(find_moving_objects ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS find_moving_objects DESTINATION lib/${PROJECT_NAME})

ament_package()

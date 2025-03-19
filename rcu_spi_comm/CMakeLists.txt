cmake_minimum_required(VERSION 3.8)
project(rcu_spi_comm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorServo.msg"
)

add_executable(spi_comm_node src/spi_comm.cpp)
ament_target_dependencies(spi_comm_node rclcpp)
add_executable(joy_to_motorservo_node src/joy_to_motorservo.cpp)
ament_target_dependencies(joy_to_motorservo_node rclcpp sensor_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(spi_comm_node "${cpp_typesupport_target}")
target_link_libraries(joy_to_motorservo_node "${cpp_typesupport_target}")

install(TARGETS
  spi_comm_node
  joy_to_motorservo_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
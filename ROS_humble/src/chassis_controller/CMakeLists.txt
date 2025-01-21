cmake_minimum_required(VERSION 3.8)
project(chassis_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
file(GLOB SERIAL_SOURCES "src/serial/*")


include_directories(
  include
  src/serial
)


add_executable(chassis_controll src/chassis_controller.cpp ${SERIAL_SOURCES})
ament_target_dependencies(chassis_controll rclcpp std_msgs)

install(
  TARGETS
  chassis_controll
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

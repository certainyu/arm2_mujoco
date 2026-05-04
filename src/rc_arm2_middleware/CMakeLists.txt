cmake_minimum_required(VERSION 3.8)
project(rc_arm2_middleware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rc_arm2_moveit_client REQUIRED)
find_package(rc_arm2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(arm2_middleware
  src/arm2_middleware.cpp
)
target_compile_features(arm2_middleware PUBLIC cxx_std_17)
ament_target_dependencies(arm2_middleware
  ament_index_cpp
  geometry_msgs
  rclcpp
  rclcpp_action
  rc_arm2_moveit_client
  rc_arm2_msgs
  std_msgs
)

install(TARGETS arm2_middleware
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

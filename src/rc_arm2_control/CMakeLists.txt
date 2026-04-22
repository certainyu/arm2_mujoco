cmake_minimum_required(VERSION 3.8)
project(rc_arm2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(arm2_torque_trajectory_controller
  src/arm2_torque_trajectory_controller.cpp
)
target_compile_features(arm2_torque_trajectory_controller PUBLIC cxx_std_17)
ament_target_dependencies(arm2_torque_trajectory_controller
  ament_index_cpp
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
)
target_link_libraries(arm2_torque_trajectory_controller
  Eigen3::Eigen
  pinocchio::pinocchio
)

install(TARGETS arm2_torque_trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/arm2_mujoco_sim.py
  scripts/send_sample_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

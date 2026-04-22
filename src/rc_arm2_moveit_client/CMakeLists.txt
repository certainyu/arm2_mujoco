cmake_minimum_required(VERSION 3.8)
project(rc_arm2_moveit_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python executable for legacy FindPythonInterp" FORCE)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('SOABI') or '')"
  OUTPUT_VARIABLE PYTHON_SOABI
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(REGEX REPLACE "^\\." "" PYTHON_SOABI "${PYTHON_SOABI}")
set(PYTHON_SOABI ${PYTHON_SOABI} CACHE INTERNAL "Python SOABI without leading dot" FORCE)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE} CACHE FILEPATH "Python executable for legacy FindPythonInterp" FORCE)
string(REGEX REPLACE "^\\." "" PYTHON_SOABI "${PYTHON_SOABI}")
set(PYTHON_SOABI ${PYTHON_SOABI} CACHE INTERNAL "Python SOABI without leading dot" FORCE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanToTaskGoal.action"
)

add_executable(arm2_plan_to_task_goal
  src/arm2_plan_to_task_goal.cpp
)
target_compile_features(arm2_plan_to_task_goal PUBLIC cxx_std_17)
ament_target_dependencies(arm2_plan_to_task_goal
  ament_index_cpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
  std_msgs
)
target_link_libraries(arm2_plan_to_task_goal
  Eigen3::Eigen
  pinocchio::pinocchio
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(arm2_plan_to_task_goal "${cpp_typesupport_target}")

add_executable(arm2_payload_scene_sync
  src/arm2_payload_scene_sync.cpp
)
target_compile_features(arm2_payload_scene_sync PUBLIC cxx_std_17)
ament_target_dependencies(arm2_payload_scene_sync
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  std_msgs
)

install(TARGETS
  arm2_plan_to_task_goal
  arm2_payload_scene_sync
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

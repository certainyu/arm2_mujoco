cmake_minimum_required(VERSION 3.8)
project(rc_arm2_description)

find_package(ament_cmake REQUIRED)

install(FILES arm2.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

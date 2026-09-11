cmake_minimum_required(VERSION 3.8)
project(rospider_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps urdf worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/sim_gait.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()

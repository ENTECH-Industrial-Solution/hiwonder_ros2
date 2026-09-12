cmake_minimum_required(VERSION 3.8)
project(rospider_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch maps rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/sim_gait.py
  DESTINATION lib/${PROJECT_NAME})

# rospider_gazebo.maps: map name -> <workspace>/maps path, used by the launch files
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # APPEND_ENV puts the package source on PYTHONPATH so `from rospider_gazebo
  # import arm_ik` resolves without depending on the install space being sourced.
  ament_add_pytest_test(test_arm_ik test/test_arm_ik.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()

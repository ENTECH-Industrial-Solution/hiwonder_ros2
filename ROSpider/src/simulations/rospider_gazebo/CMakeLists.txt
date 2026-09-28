cmake_minimum_required(VERSION 3.8)
project(rospider_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch maps models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/sim_gait.py scripts/color_detect.py scripts/pick_and_place.py
                 scripts/apriltag_detect.py scripts/yolo_detect.py
  DESTINATION lib/${PROJECT_NAME})

# The Hiwonder demo windows ported from example/ -- see launch/vision_demo.launch.py.
install(PROGRAMS scripts/apriltag_track.py scripts/object_classification.py
                 scripts/track_and_grab.py scripts/object_tracking.py
                 scripts/line_following.py
  DESTINATION lib/${PROJECT_NAME})

# rospider_gazebo.maps: map name -> <workspace>/maps path, used by the launch files
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # APPEND_ENV puts the package source on PYTHONPATH so `from rospider_gazebo
  # import arm_ik` resolves without depending on the install space being sourced.
  ament_add_pytest_test(test_arm_ik test/test_arm_ik.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_apriltag test/test_apriltag.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_labelling test/test_labelling.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_tag_settings test/test_tag_settings.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_tag_behavior test/test_tag_behavior.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_yolo_settings test/test_yolo_settings.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_lab_settings test/test_lab_settings.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_servo_map test/test_servo_map.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_pid test/test_pid.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_depth_probe test/test_depth_probe.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_shape_detect test/test_shape_detect.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_detections test/test_detections.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_color_picker test/test_color_picker.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_approach test/test_approach.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_line_follower test/test_line_follower.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()

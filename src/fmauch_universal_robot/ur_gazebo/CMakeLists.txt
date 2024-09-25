cmake_minimum_required(VERSION 2.8.3)

project(ur_gazebo)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_ur3.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur3e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur5.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur5e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur10.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur10e.xml)
  roslaunch_add_file_check(tests/roslaunch_test_ur16e.xml)
endif()
include_directories()
install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/llm.py scripts/arm_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

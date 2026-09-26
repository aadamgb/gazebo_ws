# CMake generated Testfile for 
# Source directory: /home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/manual_control
# Build directory: /home/adamgb/phd/gazebo_ws/build/px4/src/modules/manual_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(functional-ManualControl "/home/adamgb/phd/gazebo_ws/build/px4/functional-ManualControl")
set_tests_properties(functional-ManualControl PROPERTIES  _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;130;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/manual_control/CMakeLists.txt;49;px4_add_functional_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/manual_control/CMakeLists.txt;0;")
add_test(unit-ManualControlSelector "/home/adamgb/phd/gazebo_ws/build/px4/unit-ManualControlSelector")
set_tests_properties(unit-ManualControlSelector PROPERTIES  WORKING_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4" _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;72;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/manual_control/CMakeLists.txt;50;px4_add_unit_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/manual_control/CMakeLists.txt;0;")

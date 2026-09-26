# CMake generated Testfile for 
# Source directory: /home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/mc_pos_control/PositionControl
# Build directory: /home/adamgb/phd/gazebo_ws/build/px4/src/modules/mc_pos_control/PositionControl
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(unit-ControlMath "/home/adamgb/phd/gazebo_ws/build/px4/unit-ControlMath")
set_tests_properties(unit-ControlMath PROPERTIES  WORKING_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4" _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;72;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/mc_pos_control/PositionControl/CMakeLists.txt;42;px4_add_unit_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/mc_pos_control/PositionControl/CMakeLists.txt;0;")
add_test(unit-PositionControl "/home/adamgb/phd/gazebo_ws/build/px4/unit-PositionControl")
set_tests_properties(unit-PositionControl PROPERTIES  WORKING_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4" _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;72;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/mc_pos_control/PositionControl/CMakeLists.txt;43;px4_add_unit_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/mc_pos_control/PositionControl/CMakeLists.txt;0;")

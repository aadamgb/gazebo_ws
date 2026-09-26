# CMake generated Testfile for 
# Source directory: /home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/commander
# Build directory: /home/adamgb/phd/gazebo_ws/build/px4/src/modules/commander
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(unit-mag_calibration_test "/home/adamgb/phd/gazebo_ws/build/px4/unit-mag_calibration_test")
set_tests_properties(unit-mag_calibration_test PROPERTIES  WORKING_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4" _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;72;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/commander/CMakeLists.txt;81;px4_add_unit_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/commander/CMakeLists.txt;0;")
add_test(functional-ModeManagement "/home/adamgb/phd/gazebo_ws/build/px4/functional-ModeManagement")
set_tests_properties(functional-ModeManagement PROPERTIES  _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;130;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/commander/CMakeLists.txt;82;px4_add_functional_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/commander/CMakeLists.txt;0;")
subdirs("Arming")
subdirs("failsafe")
subdirs("failure_detector")
subdirs("HealthAndArmingChecks")
subdirs("ModeUtil")
subdirs("MulticopterThrowLaunch")

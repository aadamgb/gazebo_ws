# CMake generated Testfile for 
# Source directory: /home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/control_allocation/control_allocation
# Build directory: /home/adamgb/phd/gazebo_ws/build/px4/src/lib/control_allocation/control_allocation
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(unit-ControlAllocationPseudoInverse "/home/adamgb/phd/gazebo_ws/build/px4/unit-ControlAllocationPseudoInverse")
set_tests_properties(unit-ControlAllocationPseudoInverse PROPERTIES  WORKING_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4" _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;72;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/control_allocation/control_allocation/CMakeLists.txt;46;px4_add_unit_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/control_allocation/control_allocation/CMakeLists.txt;0;")
add_test(functional-ControlAllocationSequentialDesaturation "/home/adamgb/phd/gazebo_ws/build/px4/functional-ControlAllocationSequentialDesaturation")
set_tests_properties(functional-ControlAllocationSequentialDesaturation PROPERTIES  _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;130;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/control_allocation/control_allocation/CMakeLists.txt;47;px4_add_functional_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/control_allocation/control_allocation/CMakeLists.txt;0;")

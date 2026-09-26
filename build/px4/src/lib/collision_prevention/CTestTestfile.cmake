# CMake generated Testfile for 
# Source directory: /home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/collision_prevention
# Build directory: /home/adamgb/phd/gazebo_ws/build/px4/src/lib/collision_prevention
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(functional-CollisionPrevention "/home/adamgb/phd/gazebo_ws/build/px4/functional-CollisionPrevention")
set_tests_properties(functional-CollisionPrevention PROPERTIES  _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;130;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/collision_prevention/CMakeLists.txt;42;px4_add_functional_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/collision_prevention/CMakeLists.txt;0;")
add_test(unit-ObstacleMath "/home/adamgb/phd/gazebo_ws/build/px4/unit-ObstacleMath")
set_tests_properties(unit-ObstacleMath PROPERTIES  WORKING_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4" _BACKTRACE_TRIPLES "/home/adamgb/phd/gazebo_ws/src/px4_firmware/cmake/px4_add_gtest.cmake;72;add_test;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/collision_prevention/CMakeLists.txt;43;px4_add_unit_gtest;/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/lib/collision_prevention/CMakeLists.txt;0;")

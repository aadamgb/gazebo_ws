# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/adamgb/phd/gazebo_ws/src/px4_firmware/Tools/simulation/gz"
  "/home/adamgb/phd/gazebo_ws/build/px4/build_gz"
  "/home/adamgb/phd/gazebo_ws/build/px4/external/Install/gz"
  "/home/adamgb/phd/gazebo_ws/build/px4/external/tmp/gz"
  "/home/adamgb/phd/gazebo_ws/build/px4/external/Stamp/gz"
  "/home/adamgb/phd/gazebo_ws/build/px4/external/Download/gz"
  "/home/adamgb/phd/gazebo_ws/build/px4/external/Stamp/gz"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4/external/Stamp/gz/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4/external/Stamp/gz${cfgdir}") # cfgdir has leading slash
endif()

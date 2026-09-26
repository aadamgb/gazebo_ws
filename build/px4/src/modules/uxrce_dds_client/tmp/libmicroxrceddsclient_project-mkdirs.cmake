# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/adamgb/phd/gazebo_ws/src/px4_firmware/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/tmp"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-stamp"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-stamp${cfgdir}") # cfgdir has leading slash
endif()

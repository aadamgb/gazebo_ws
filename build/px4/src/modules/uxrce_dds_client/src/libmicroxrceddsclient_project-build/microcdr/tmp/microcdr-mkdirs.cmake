# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-build"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/temp_install"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/tmp"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src"
  "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/adamgb/phd/gazebo_ws/build/px4/src/modules/uxrce_dds_client/src/libmicroxrceddsclient_project-build/microcdr/src/microcdr-stamp${cfgdir}") # cfgdir has leading slash
endif()

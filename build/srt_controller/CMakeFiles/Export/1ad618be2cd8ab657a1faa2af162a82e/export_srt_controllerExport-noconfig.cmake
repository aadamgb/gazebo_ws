#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "srt_controller::SrtController_ActuatorsController" for configuration ""
set_property(TARGET srt_controller::SrtController_ActuatorsController APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(srt_controller::SrtController_ActuatorsController PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "rclcpp::rclcpp;mrs_lib::mrs_lib;mrs_msgs::mrs_msgs__rosidl_generator_c;mrs_msgs::mrs_msgs__rosidl_typesupport_fastrtps_c;mrs_msgs::mrs_msgs__rosidl_typesupport_fastrtps_cpp;mrs_msgs::mrs_msgs__rosidl_typesupport_introspection_c;mrs_msgs::mrs_msgs__rosidl_typesupport_c;mrs_msgs::mrs_msgs__rosidl_typesupport_introspection_cpp;mrs_msgs::mrs_msgs__rosidl_typesupport_cpp;mrs_msgs::mrs_msgs__rosidl_generator_py"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libSrtController_ActuatorsController.so"
  IMPORTED_SONAME_NOCONFIG "libSrtController_ActuatorsController.so"
  )

list(APPEND _cmake_import_check_targets srt_controller::SrtController_ActuatorsController )
list(APPEND _cmake_import_check_files_for_srt_controller::SrtController_ActuatorsController "${_IMPORT_PREFIX}/lib/libSrtController_ActuatorsController.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

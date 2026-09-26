#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lvhr_controller::LvhrController_VelocityController" for configuration ""
set_property(TARGET lvhr_controller::LvhrController_VelocityController APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(lvhr_controller::LvhrController_VelocityController PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "rclcpp::rclcpp;mrs_lib::mrs_lib;mrs_msgs::mrs_msgs__rosidl_generator_c;mrs_msgs::mrs_msgs__rosidl_typesupport_fastrtps_c;mrs_msgs::mrs_msgs__rosidl_typesupport_fastrtps_cpp;mrs_msgs::mrs_msgs__rosidl_typesupport_introspection_c;mrs_msgs::mrs_msgs__rosidl_typesupport_c;mrs_msgs::mrs_msgs__rosidl_typesupport_introspection_cpp;mrs_msgs::mrs_msgs__rosidl_typesupport_cpp;mrs_msgs::mrs_msgs__rosidl_generator_py;geometry_msgs::geometry_msgs__rosidl_generator_c;geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_c;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_c;geometry_msgs::geometry_msgs__rosidl_typesupport_c;geometry_msgs::geometry_msgs__rosidl_typesupport_fastrtps_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_introspection_cpp;geometry_msgs::geometry_msgs__rosidl_typesupport_cpp;geometry_msgs::geometry_msgs__rosidl_generator_py;ament_index_cpp::ament_index_cpp;yaml-cpp::yaml-cpp"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libLvhrController_VelocityController.so"
  IMPORTED_SONAME_NOCONFIG "libLvhrController_VelocityController.so"
  )

list(APPEND _cmake_import_check_targets lvhr_controller::LvhrController_VelocityController )
list(APPEND _cmake_import_check_files_for_lvhr_controller::LvhrController_VelocityController "${_IMPORT_PREFIX}/lib/libLvhrController_VelocityController.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

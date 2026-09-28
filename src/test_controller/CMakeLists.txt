cmake_minimum_required(VERSION 3.28)
project(test_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_uav_managers REQUIRED)
find_package(mrs_msgs REQUIRED)

add_library(TestController_TestController SHARED src/test_controller.cpp)

target_link_libraries(TestController_TestController
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_managers::mrs_uav_managers
    ${mrs_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

install(TARGETS TestController_TestController
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

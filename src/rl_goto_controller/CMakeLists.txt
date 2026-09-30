cmake_minimum_required(VERSION 3.28)
project(rl_goto_controller)

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
find_package(Eigen3 REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# libtorch is linked directly (CPU only), only used to load the TorchScript policy
set(LIBTORCH_DIR "$ENV{HOME}/miniconda3/envs/rl/lib/python3.10/site-packages/torch"
  CACHE PATH "Root of the libtorch install (contains lib/ and include/)")

find_library(TORCH_CPU_LIBRARY torch_cpu PATHS "${LIBTORCH_DIR}/lib" NO_DEFAULT_PATH)
find_library(C10_LIBRARY       c10       PATHS "${LIBTORCH_DIR}/lib" NO_DEFAULT_PATH)

if(NOT TORCH_CPU_LIBRARY OR NOT C10_LIBRARY)
  message(FATAL_ERROR "[rl_goto_controller]: libtorch not found in LIBTORCH_DIR='${LIBTORCH_DIR}'. "
                      "Pass -DLIBTORCH_DIR=<path to torch/libtorch dir>.")
endif()

add_library(RlGotoController_RLGoto SHARED src/rl_goto.cpp)

target_include_directories(RlGotoController_RLGoto SYSTEM
  PRIVATE
    "${LIBTORCH_DIR}/include"
    "${LIBTORCH_DIR}/include/torch/csrc/api/include"
)

target_link_libraries(RlGotoController_RLGoto
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_managers::mrs_uav_managers
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ament_index_cpp::ament_index_cpp
    ${TORCH_CPU_LIBRARY}
    ${C10_LIBRARY}
)

# control_manager dlopen()s this plugin, so it has to find libtorch on its own
set_target_properties(RlGotoController_RLGoto PROPERTIES
  BUILD_RPATH   "${LIBTORCH_DIR}/lib"
  INSTALL_RPATH "${LIBTORCH_DIR}/lib"
)

pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

install(TARGETS RlGotoController_RLGoto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.28)
project(lvhr_controller)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

# Find all dependencies of this package
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mrs_lib REQUIRED)
find_package(mrs_uav_managers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

## --------------------------------------------------------------
## |                          libtorch                          |
## --------------------------------------------------------------

# find_package(Torch) is deliberately NOT used: the torch in the `rl` conda env
# is a CUDA build, and its Caffe2Config.cmake hard-fails at configure time
# unless the full CUDA toolkit (nvcc + headers) is installed. The policy runs on
# the CPU, so we link libtorch_cpu + libc10 directly and never touch CUDA.
#
# LIBTORCH_DIR may point at either the `torch` dir of a pip/conda install or an
# unzipped standalone libtorch -- both have the same lib/ + include/ layout:
#   colcon build --cmake-args -DLIBTORCH_DIR=/usr/local/lib/libtorch
set(LIBTORCH_DIR "$ENV{HOME}/miniconda3/envs/rl/lib/python3.10/site-packages/torch"
  CACHE PATH "Root of the libtorch install (contains lib/ and include/)")

find_library(TORCH_CPU_LIBRARY torch_cpu PATHS "${LIBTORCH_DIR}/lib" NO_DEFAULT_PATH)
find_library(C10_LIBRARY       c10       PATHS "${LIBTORCH_DIR}/lib" NO_DEFAULT_PATH)

if(NOT TORCH_CPU_LIBRARY OR NOT C10_LIBRARY)
  message(FATAL_ERROR "[lvhr_controller]: libtorch not found in LIBTORCH_DIR='${LIBTORCH_DIR}'. "
                      "Pass -DLIBTORCH_DIR=<path to torch/libtorch dir>.")
endif()

message(STATUS "[lvhr_controller]: using libtorch from ${LIBTORCH_DIR}")

## --------------------------------------------------------------
## |                 LVHR SO(3) velocity controller             |
## --------------------------------------------------------------

add_library(LvhrController_VelocityController SHARED)

target_sources(LvhrController_VelocityController
  PRIVATE
    "src/velocity_controller.cpp"
)

# SYSTEM: libtorch headers trip -Wall/-Wextra/-Wpedantic by the hundreds
target_include_directories(LvhrController_VelocityController SYSTEM
  PRIVATE
    "${LIBTORCH_DIR}/include"
    "${LIBTORCH_DIR}/include/torch/csrc/api/include"
)

target_link_libraries(LvhrController_VelocityController
  PRIVATE
    rclcpp::rclcpp
    pluginlib::pluginlib
    mrs_lib::mrs_lib
    mrs_uav_managers::mrs_uav_managers
    Eigen3::Eigen
    ${mrs_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ament_index_cpp::ament_index_cpp
    yaml-cpp::yaml-cpp
    ${TORCH_CPU_LIBRARY}
    ${C10_LIBRARY}
)

# control_manager dlopen()s this plugin, so it has to be able to find libtorch
# on its own. libtorch's own deps (libgomp, the CUDA runtime) are resolved
# through libtorch's RPATH, so only the torch lib dir needs to be listed here.
set_target_properties(LvhrController_VelocityController PROPERTIES
  BUILD_RPATH   "${LIBTORCH_DIR}/lib"
  INSTALL_RPATH "${LIBTORCH_DIR}/lib"
)


## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(mrs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS LvhrController_VelocityController
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_package()

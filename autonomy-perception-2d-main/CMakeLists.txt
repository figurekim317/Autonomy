cmake_minimum_required(VERSION 3.5)
project(perception_2d)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(" * Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(CMAKE_BUILD_TYPE MATCHES Release)
  message(" * Release Mode")
  add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  message(" * Release with Debug Info Mode")
  add_compile_options(-Wno-deprecated-declarations)
endif()

if(CMAKE_BUILD_TYPE MATCHES Debug)
  message(" * Debug Mode")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CUDA REQUIRED)
find_package(autonomy_ros2_message REQUIRED)
find_package(parameter_manager REQUIRED)

find_package(common_api REQUIRED)
find_package(deepstream_manager REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

# include directory
include_directories(
  include
  /opt/nvidia/deepstream/deepstream/sources/includes
  ${CUDA_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${parameter_manager_INCLUDE_DIRS}

  ${common_api_INCLUDE_DIRS}
  ${deepstream_manager_INCLUDE_DIRS}
)

link_directories(
  /opt/nvidia/deepstream/deepstream/lib/
)

# cpp executable
add_executable(perception_2d src/nvds_2d_perception.cpp src/neu_2d_perception_core.cpp src/camera_calibration/inverse_perspective_mapping.cpp)

# Install Cpp executables
install(TARGETS
  perception_2d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  services
  resource
  DESTINATION share/${PROJECT_NAME}/
)

set(dependencies
  rclcpp
  sensor_msgs
  autonomy_ros2_message
  std_msgs
  GST
  ament_index_cpp
  OpenCV
)

set(all_gstreamer_libraries
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER-APP_LIBRARIES}
  ${GSTREAMER-AUDIO_LIBRARIES}
  ${GSTREAMER-PBUTILS_LIBRARIES}
  ${GSTREAMER-FFT_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvdsgst_helper
  nvdsgst_meta
  nvds_meta
  nvbufsurface
  nvbufsurftransform
)

ament_target_dependencies(perception_2d
  ${dependencies}
)

target_link_libraries(perception_2d
  ${all_gstreamer_libraries}
  ${parameter_manager_LIBRARIES}

  ${common_api_LIBRARIES}
  ${deepstream_manager_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

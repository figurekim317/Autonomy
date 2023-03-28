cmake_minimum_required(VERSION 3.5)
project(deepstream_manager)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(CUDA REQUIRED)
find_package(parameter_manager REQUIRED)
find_package(common_api REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)

pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

include_directories(
  include
  /opt/nvidia/deepstream/deepstream/sources/includes
  ${CUDA_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${parameter_manager_INCLUDE_DIRS}
  ${common_api_INCLUDE_DIRS}
)

link_directories(
  /opt/nvidia/deepstream/deepstream/lib/
)

set(LIB_SET
  rclcpp
  OpenCV
  parameter_manager
  common_api
  ament_index_cpp
  CUDA
  PkgConfig
)

add_library(${PROJECT_NAME}_export SHARED
  src/neu_2d_img_subs.cpp
  src/neu_deepstream_core.cpp
  src/pipeline/deepstream_manager_pipeline.cpp
  src/pipeline/pipeline_parameter.cpp
  src/neu_perception_2d.cpp
  src/pipeline/perception_2d_parameter.cpp
)

ament_target_dependencies(${PROJECT_NAME}_export
  ${LIB_SET}
)

install(DIRECTORY
  engine
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME}_export)
ament_export_include_directories(include)
ament_package()

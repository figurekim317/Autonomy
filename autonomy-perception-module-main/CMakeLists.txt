cmake_minimum_required(VERSION 3.5)
project(perception_module)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# # Add support for C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)

  # set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(perception_pcl REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autonomy_ros2_message REQUIRED)
find_package(parameter_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_api REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include
  ${PCL_INCLUDE_DIRS}
)

set(lib_set
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  perception_pcl
  visualization_msgs
  autonomy_ros2_message
  parameter_manager
  geometry_msgs
  OpenCV
  cv_bridge
  image_transport
  nav_msgs
  common_api
)

add_library(${PROJECT_NAME}_export SHARED
  src/range_util.cpp
  src/image_dimension_transfomration/transformer.cpp
)

ament_target_dependencies(${PROJECT_NAME}_export
  ${lib_set}
)

set(OBJECT_DETECTION_FUSION_SRC
  src/object_detection_fusion/class_fusion.cpp
  src/object_detection_fusion/class_fusion_node.cpp
)

add_executable(class_fusion src/object_detection_fusion/class_fusion_node.cpp src/object_detection_fusion/class_fusion.cpp)
ament_target_dependencies(class_fusion ${lib_set})
target_link_libraries(class_fusion ${PCL_LIBRARIES} ${parameter_manager_LIBRARIES} ${PROJECT_NAME}_export)

set(SEGMENTATION_FUSION_SRC
  src/segmentation_fusion/compare_point.cpp
  src/segmentation_fusion/erosion_dilation.cpp
  src/segmentation_fusion/segmentation_fusion_node.cpp
)

add_executable(segmentation_fusion ${SEGMENTATION_FUSION_SRC})
ament_target_dependencies(segmentation_fusion ${lib_set})
target_link_libraries(segmentation_fusion ${PCL_LIBRARIES} ${PROJECT_NAME}_export)

set(VISUALIZER_SRC
  src/test/image_visualizer.cpp
  src/test/bbox_visualizer.cpp
  src/test/obstacles_visualizer.cpp
  src/test/segmentation_visualizer.cpp
  src/test/publisher.cpp
  src/test/visualizer_node.cpp
)

add_executable(perception_visualizer ${VISUALIZER_SRC})
ament_target_dependencies(perception_visualizer ${lib_set})
target_link_libraries(perception_visualizer ${PCL_LIBRARIES} ${PROJECT_NAME}_export)

set(RADAR_FUSION_SRC
  src/radar_fusion/radar_fusion.cpp
  src/radar_fusion/radar_fusion_node.cpp
)

add_executable(radar_fusion src/radar_fusion/radar_fusion_node.cpp src/radar_fusion/radar_fusion.cpp)
ament_target_dependencies(radar_fusion ${lib_set})
target_link_libraries(radar_fusion ${PCL_LIBRARIES} ${PROJECT_NAME}_export)

add_executable(detection_transformer src/image_dimension_transfomration/detection_transformer_node.cpp src/image_dimension_transfomration/detection_transformer.cpp)
ament_target_dependencies(detection_transformer ${lib_set})
target_link_libraries(detection_transformer ${PROJECT_NAME}_export Boost::system)

add_executable(panic_mode_detector src/panic_mode/panic_mode_detector_node.cpp src/panic_mode/panic_mode_detector.cpp)
ament_target_dependencies(panic_mode_detector ${lib_set})
target_link_libraries(panic_mode_detector ${PCL_LIBRARIES} ${PROJECT_NAME}_export)

install(TARGETS
  ${PROJECT_NAME}_export
  segmentation_fusion
  class_fusion
  perception_visualizer
  radar_fusion
  detection_transformer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  set(REALSENSE_DEVICE_PLUGIN FALSE)

  if(${REALSENSE_DEVICE_PLUGIN})
    ament_add_gtest(test_api test/test_api.cpp)
  endif()

  if(TARGET test_api)
    target_include_directories(test_api PUBLIC
      ${${PROJECT_NAME}_INCLUDE_DIRS}
    )
  endif()
endif()

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(${PROJECT_NAME}_export)
ament_export_include_directories(include)
ament_package()

cmake_minimum_required(VERSION 3.5)
project(3d_perception)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
## Add support for C++11, supported in ROS Kinetic and newer
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  # set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()




find_package(PCL REQUIRED)


include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${parameter_manager_INCLUDE_DIRS}
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
  common_api
  OpenCV
)

add_library(UTIL_HEADER SHARED
   src/util/range_util.cpp
   src/util/dbscan_phase.cpp
)

ament_target_dependencies(UTIL_HEADER
  ${lib_set}
)

# set(UTIL_SRC 
#   src/util/point_rotation_node.cpp
#   src/util/point_rotation.cpp
#   src/util/range_region_node.cpp
#   src/util/range_region.cpp
# )

add_executable(point_rotation src/util/point_rotation_node.cpp src/util/point_rotation.cpp )
ament_target_dependencies(point_rotation ${lib_set})
target_link_libraries(point_rotation ${PCL_LIBRARIES} UTIL_HEADER ${parameter_manager_LIBRARIES})

add_executable(range_region src/util/range_region_node.cpp src/util/range_region.cpp )
ament_target_dependencies(range_region ${lib_set})
target_link_libraries(range_region ${PCL_LIBRARIES} UTIL_HEADER ${parameter_manager_LIBRARIES})

set(COMMON_SRC 
  src/common/clustering_point.cpp
)

set(OBSTACLES_SRC 
  src/obstacles_detection/combine_object.cpp
  src/obstacles_detection/post_filtering.cpp
  src/obstacles_detection/obstales_detection_node.cpp
)

add_executable(obstacles_detection ${OBSTACLES_SRC} ${COMMON_SRC})
ament_target_dependencies(obstacles_detection ${lib_set})
target_link_libraries(obstacles_detection ${PCL_LIBRARIES} UTIL_HEADER ${parameter_manager_LIBRARIES})

set(CURB_SRC 
  src/curb_detection/clustering_curb.cpp
  src/curb_detection/combine_ground.cpp
  src/curb_detection/curb_detection_node.cpp
)

add_executable(curb_detection ${CURB_SRC} ${COMMON_SRC})
ament_target_dependencies(curb_detection ${lib_set})
target_link_libraries(curb_detection ${PCL_LIBRARIES} UTIL_HEADER ${parameter_manager_LIBRARIES})

set(GROUND_SRC
  src/curb_detection/combine_ground.cpp
  src/curb_detection/combine_ground_for_segmentation_node.cpp
)

add_executable(combine_ground ${GROUND_SRC})
ament_target_dependencies(combine_ground ${lib_set})
target_link_libraries(combine_ground ${PCL_LIBRARIES} UTIL_HEADER ${parameter_manager_LIBRARIES})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS 
  UTIL_HEADER
  point_rotation
  range_region
  obstacles_detection
  curb_detection
  combine_ground
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

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
ament_export_libraries(UTIL_HEADER)
ament_export_include_directories(include)
ament_package()

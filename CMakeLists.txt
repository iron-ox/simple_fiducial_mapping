cmake_minimum_required(VERSION 2.8.3)
project(simple_fiducial_mapping)

find_package(catkin REQUIRED COMPONENTS
  apriltags2_ros
  cv_bridge
  geometry_msgs
  image_geometry
  robot_localization
  rosbag
  roscpp
  tf2_ros
  tf2_eigen
  visualization_msgs
  )
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS apriltags2_ros geometry_msgs visualization_msgs
  LIBRARIES ${PROJECT_NAME}_mapping
  DEPENDS Eigen3 OpenCV
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

add_compile_options(-std=c++1z)

add_library(${PROJECT_NAME}_mapping
  src/mapping.cpp src/visualizer.cpp
  src/cv_utils.cpp src/map_graph.cpp
  src/ceres_backend.cpp
  )
add_dependencies(${PROJECT_NAME}_mapping ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mapping yaml-cpp ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES})

add_executable(${PROJECT_NAME}_map_from_bag src/map_from_bag.cpp)
target_link_libraries(${PROJECT_NAME}_map_from_bag ${PROJECT_NAME}_mapping ${catkin_LIBRARIES} ${CERES_LIBRARIES})
set_target_properties(${PROJECT_NAME}_map_from_bag PROPERTIES OUTPUT_NAME map_from_bag PREFIX "")

add_executable(${PROJECT_NAME}_localization_node src/localization_node_main.cpp src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_localization_node ${PROJECT_NAME}_mapping ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_localization_node PROPERTIES OUTPUT_NAME localization_node PREFIX "")

add_executable(${PROJECT_NAME}_localize_from_bag src/localize_from_bag.cpp src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_localize_from_bag ${PROJECT_NAME}_mapping yaml-cpp ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_localize_from_bag PROPERTIES OUTPUT_NAME localize_from_bag PREFIX "")

install(
  TARGETS
  ${PROJECT_NAME}_mapping ${PROJECT_NAME}_map_from_bag
  ${PROJECT_NAME}_localization_node ${PROJECT_NAME}_localize_from_bag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rosunit REQUIRED)
  catkin_add_gtest(mapping_test test/mapping_test.cpp)
  target_link_libraries(mapping_test ${PROJECT_NAME}_mapping ${catkin_LIBRARIES})
endif()

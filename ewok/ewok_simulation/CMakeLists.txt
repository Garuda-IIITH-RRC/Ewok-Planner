cmake_minimum_required(VERSION 2.8.3)
project(ewok_simulation)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(-std=c++11 -g -O3)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  octomap_msgs
  octomap_ros
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
  std_msgs
  visualization_msgs
)


include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})

catkin_simple()

cs_add_executable(bspline_lee_position_controller_node src/bspline_lee_position_controller_node.cpp)
cs_add_executable(trajectory_replanning_example src/trajectory_replanning_example.cpp)

cs_add_executable(trajectory_replanning_pointcloud src/trajectory_replanning_example_pointCloud.cpp)


cs_add_executable(trajectory_replanning_rishabh src/trajectory_replanning_rishabh.cpp)
cs_add_executable(bspline_controller_rishabh src/bspline_controller_rishabh.cpp)

add_executable(pubCamPose src/pubCamPose.cpp)
target_link_libraries(pubCamPose ${catkin_LIBRARIES})


add_executable(pcNoise src/pcNoise.cpp)
target_link_libraries(pcNoise ${catkin_LIBRARIES})


target_link_libraries(trajectory_replanning_rishabh ${CHOLMOD_LIBRARY} nlopt)
target_link_libraries(trajectory_replanning_pointcloud ${CHOLMOD_LIBRARY} nlopt)



target_link_libraries(trajectory_replanning_example ${CHOLMOD_LIBRARY} nlopt)

cs_install()
cs_export()

cmake_minimum_required(VERSION 2.8.3)
project(kinect_slam)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  message_generation
  tf2_ros
  pcl_ros
  cv_bridge
  dynamic_reconfigure
  image_transport
  image_geometry
  geometry_msgs
  roscpp
)

add_message_files(
  FILES
  LandmarkMsg.msg
)

generate_dynamic_reconfigure_options(
  cfg/EKFSLAMConfig.cfg
  cfg/LandmarkExtractorConfig.cfg
)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(slam_node src/ekf_slam_node_main.cpp src/EKF_SLAM_Node.cpp src/EKF_SLAM.cpp)
add_executable(landmark_node src/LandmarkExtractorNode_main.cpp src/LandmarkExtractorNode.cpp src/HarrisDetector.cpp src/BRIEF.cpp)
add_executable(control_node src/control_main.cpp src/Control_Node.cpp src/Control.cpp)

add_dependencies(slam_node ${PROJECT_NAME}_gencfg)
add_dependencies(landmark_node ${PROJECT_NAME}_gencfg)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

target_link_libraries(slam_node ${catkin_LIBRARIES})
target_link_libraries(landmark_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(control_node
   ${catkin_LIBRARIES}
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf
    CATKIN_DEPENDS message_runtime
)

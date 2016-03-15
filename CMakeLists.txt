cmake_minimum_required(VERSION 2.8.3)
project(kinect_slam)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  pcl_ros
  cv_bridge
  dynamic_reconfigure
  image_transport
  image_geometry
  geometry_msgs
  roscpp
)

generate_dynamic_reconfigure_options(
  cfg/KinectSLAMConfig.cfg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(landmark_matcher src/landmark_matcher_main.cpp src/LandmarkMatcherNode.cpp src/BRIEF.cpp src/HarrisDetector.cpp src/BRIEF.cpp src/StereoMatcher.cpp)

add_dependencies(landmark_matcher ${PROJECT_NAME}_gencfg)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
target_link_libraries(landmark_matcher ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})


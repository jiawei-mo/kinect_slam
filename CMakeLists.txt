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
  BumperState.msg
  PioneerVelControl.msg
)

generate_dynamic_reconfigure_options(
  cfg/KinectSLAMConfig.cfg
  cfg/RosAria.cfg
)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

find_package(Aria QUIET)
if(Aria_FOUND)
  if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
    add_definitions(-DADEPT_PKG)
  endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
  if(EXISTS "/usr/local/Aria/include/Aria.h")
    add_definitions(-DADEPT_PKG)
    include_directories( /usr/local/Aria/include)
    link_directories(/usr/local/Aria/lib)
  endif()
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(slam_node src/ekf_slam_node_main.cpp src/EKF_SLAM_Node.cpp src/EKF_SLAM.cpp)
add_executable(kinect_node src/LandmarkExtractorNode_main.cpp src/LandmarkExtractorNode.cpp src/HarrisDetector.cpp src/BRIEF.cpp)

add_executable(aria src/RosAria.cpp src/LaserPublisher.cpp)
add_dependencies(aria rosaria_gencfg)
add_dependencies(aria rosaria_gencpp)
add_dependencies(slam_node ${PROJECT_NAME}_gencfg)
add_dependencies(kinect_node ${PROJECT_NAME}_gencfg)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
target_link_libraries(slam_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(kinect_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(aria ${catkin_LIBRARIES} ${Boost_LIBRARIES} Aria pthread dl rt)
set_target_properties(aria PROPERTIES COMPILE_FLAGS "-fPIC")

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
catkin_package(
    DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf
    CATKIN_DEPENDS message_runtime
)

install(
  TARGETS 
    aria
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

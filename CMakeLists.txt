cmake_minimum_required(VERSION 3.0.2)
project(realsense2_video_capture)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge 
  image_transport
  message_filters
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${BOOST_LIBRARIES})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARIES})

add_executable(video_capture_node 
src/video_capture_node.cpp
)
add_dependencies(video_capture_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(video_capture_node ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(StereoVision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation

)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})


add_executable(stereo /home/usera/catkin_ws/src/taehyeong/StereoVision/Cpp/StereoVision.cpp /home/usera/catkin_ws/src/taehyeong/StereoVision/Cpp/main.cpp)
target_link_libraries(stereo ${catkin_LIBRARIES})
target_link_libraries(stereo ${OpenCV_LIBS})

add_executable(pub /home/usera/catkin_ws/src/taehyeong/StereoVision/Cpp/vision_example_publish.cpp)
target_link_libraries(pub ${catkin_LIBRARIES})
target_link_libraries(pub ${OpenCV_LIBS})

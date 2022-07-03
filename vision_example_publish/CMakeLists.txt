cmake_minimum_required(VERSION 2.8.3)
project(vision_example_publish)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation

)
find_package(OpenCV REQUIRED)

add_message_files(FILES stopmsg.msg)

generate_messages(DEPENDENCIES std_msgs)

include_directories(${OpenCV_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(vision_example_publish src/vision_example_publish.cpp)
target_link_libraries(vision_example_publish ${catkin_LIBRARIES})
target_link_libraries(vision_example_publish ${OpenCV_LIBS})

add_executable(parking_test_0703 src/parking_test_0703.cpp)
target_link_libraries(parking_test_0703 ${catkin_LIBRARIES} ${OpenCV_LIBS})


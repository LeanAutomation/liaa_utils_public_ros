cmake_minimum_required(VERSION 2.8.3)
project(tf_topic_tools)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

catkin_package(
  CATKIN_DEPENDS roscpp  std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_to_topic src/tf_to_topic.cpp)
target_link_libraries(tf_to_topic ${catkin_LIBRARIES})

add_executable(topic_to_tf src/topic_to_tf.cpp)
target_link_libraries(topic_to_tf ${catkin_LIBRARIES})

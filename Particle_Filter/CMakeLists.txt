cmake_minimum_required(VERSION 2.8.3)
project(particle_filter)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge genmsg image_transport sensor_msgs geometry_msgs tf gazebo_msgs)

catkin_package()

find_package(OpenCV)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_truth_publisher src/ground_truth_publisher.cpp)
target_link_libraries(ground_truth_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(localizer_node src/localizer_node.cpp)
target_link_libraries(localizer_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_compile_options(localizer_node PRIVATE -std=c++11)

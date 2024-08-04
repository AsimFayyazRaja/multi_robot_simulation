cmake_minimum_required(VERSION 3.0.2)
project(xy_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)



set(CMAKE_CXX_STANDARD 11)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV  REQUIRED)


add_executable(main src/main.cpp)

target_link_libraries(main PRIVATE robots map visualize lidar ${catkin_LIBRARIES} ${OpenCV_LIBS} )
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libs)

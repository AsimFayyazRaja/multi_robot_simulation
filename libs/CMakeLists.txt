cmake_minimum_required(VERSION 3.12)
project(xy_simulator)

set(CMAKE_CXX_STANDARD 11)


find_package(yaml-cpp REQUIRED)
find_package(OpenCV  REQUIRED)

add_library(robots SHARED robots.cpp)
add_library(map SHARED map.cpp)
add_library(visualize SHARED visualize.cpp)
add_library(lidar SHARED lidar.cpp)


target_link_libraries(robots PUBLIC ${YAML_CPP_LIBRARIES} )
target_include_directories(robots PUBLIC ${YAML_CPP_INCLUDE_DIR})

target_link_libraries(map PUBLIC ${YAML_CPP_LIBRARIES} )
target_include_directories(map PUBLIC ${YAML_CPP_INCLUDE_DIR})

target_link_libraries(visualize PUBLIC ${YAML_CPP_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(visualize PUBLIC ${YAML_CPP_INCLUDE_DIR})

target_link_libraries(lidar PUBLIC  ${OpenCV_LIBS})

target_include_directories(robots PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

include_directories( ${OpenCV_INCLUDE_DIRS})




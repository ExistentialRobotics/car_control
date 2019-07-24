cmake_minimum_required(VERSION 2.8.3)
project(car_control)

add_compile_options(-std=c++11 -Wall)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs tf)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp nav_msgs std_msgs tf
  DEPENDS EIGEN3)

add_library(DDControl src/DDControl.cpp)
add_library(power_diagram_control src/power_diagram_control.cpp)

add_executable(dd_control_node src/dd_control_node.cpp)
add_executable(power_diagram_node src/power_diagram_node.cpp)
add_dependencies(dd_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dd_control_node DDControl ${catkin_LIBRARIES})
add_dependencies(power_diagram_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(power_diagram_node power_diagram_control DDControl ${catkin_LIBRARIES})


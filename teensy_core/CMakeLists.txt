cmake_minimum_required(VERSION 3.0.2)
project(teensy_core)

# set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  daedalus_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES teensy_core
  CATKIN_DEPENDS 
    roscpp 
    rospy
    daedalus_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(teensy_core 
    src/teensy_comm.cpp  # Add this line
)

add_executable(teensy_interface src/teensy_interface.cpp)
target_link_libraries(teensy_interface ${catkin_LIBRARIES} teensy_core)
add_dependencies(teensy_interface ${catkin_EXPORTED_TARGETS})

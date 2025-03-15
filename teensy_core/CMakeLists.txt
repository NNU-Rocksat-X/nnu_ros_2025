cmake_minimum_required(VERSION 3.0.2)
project(teensy_core)

add_compile_options(-std=c++17)
add_compile_options(-Wno-psabi) # remove annoying warning about vectors



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
    src/move_manager.cpp
)

add_executable(teensy_interface src/teensy_interface.cpp)
target_link_libraries(teensy_interface ${catkin_LIBRARIES} teensy_core)
add_dependencies(teensy_interface ${catkin_EXPORTED_TARGETS})

add_executable(move_manager src/move_manager.cpp)
target_link_libraries(move_manager ${catkin_LIBRARIES} teensy_core)
add_dependencies(move_manager ${catkin_EXPORTED_TARGETS})
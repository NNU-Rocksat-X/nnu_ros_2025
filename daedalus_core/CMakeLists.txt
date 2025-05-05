cmake_minimum_required(VERSION 2.8.3)
project(daedalus_core)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
)

# Ensure Python modules are available
set(PYTHON_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(ENV{PYTHONPATH} "${PYTHON_PATH}:$ENV{PYTHONPATH}")

install(DIRECTORY src/daedalus_core
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_install_python(
    PROGRAMS
    nodes/mission_state_machine
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

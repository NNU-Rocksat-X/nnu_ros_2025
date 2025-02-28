cmake_minimum_required(VERSION 3.0.2)
project(daedalus_msgs)

find_package(catkin REQUIRED 
    COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        genmsg
        actionlib_msgs
)

add_message_files(
    FILES
    teensy_message.msg
)
  
# add_service_files(
#     FILES
# )


generate_messages(
    DEPENDENCIES
)

catkin_package(
    
)
cmake_minimum_required(VERSION 3.0.2)
project(daedalus_msgs)

find_package(catkin REQUIRED 
    COMPONENTS
        message_generation
)

add_message_files(
    FILES
    teensy_message.msg
)
  
add_service_files(
    FILES
    joint_pose_cmd.srv
    inhibit_detection.srv
)


generate_messages(
    DEPENDENCIES
)

catkin_package(
    
)
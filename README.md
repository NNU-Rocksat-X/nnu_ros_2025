# nnu_ros

This is a rebuilt ROS codebase for the 2024-2025 NNU Rocksat Mission. The
complexity has been stripped out and basic functionaltiy has been set up to 
ensure mission success. 

If you haven't already you should lean into learning git as you work on this 
project.

# Code Basics

The basic parts are the state machine, the joint pose service, and the teensy 
interface. The mission state machine controls the mission timeline and monitors
for the timer events. The inhibits are also controlled at the beginning of the
mission state machine. It uses a ros library called smach that makes it super
easy to make hierarchical state machines. The joint pose service is a ros 
service that acts as a bridge between the state machine and the teensy_interface.
It pulls the corresponding parameter from the ros params and then publishes a 
message to the teensy interface and then it waits for the teensy interface to 
return encoder values that are within the tolerance of the completed movement. 
The teensy interface is simply a manager for the serial communication between 
the teensy and the jetson. If you want to learn more about serial communication,
ask Dr. Griffin (he's kina an expert on it).

Here's a basic outline of the repo
```
- nnu_ros
    - daedalus_msgs
        - msg
            - (this is where the ros publisher/subscriber messages are defined)
        - srv
            - (this is where the ros service messages are defined)
    - daedalus_core
        - config
            - (this is for parameters that will be pulled into ros at the 
               beginning of the mission)
        - include (put header files here)
        - launch 
            - (this is where the ros launch files go. For this project each node
               gets his own launch file that the main mission launch file will
               interface with. Launch files for test configurations are also
               good to make.)
        - nodes
            - (this is where the python ros nodes go. (the .py extension is 
               left off by ros convention))
        - src (put cpp files here)
    - teensy_core
        - include
        - launch
        - src
```

In each of these directories is a CMakeLists.txt and a package.xml. ROS uses 
a set of CMake macros (catkin) to make CMake interface with the ros system a 
little better. Do some reading on cmake if you are interested.

Any time you change the c++ code, you need to rebuild the project. To build, 
move into `~/catkin_ws` and run `catkin_make`. To do a clean build, run 
`catkin_make clean` (this removes the executables and rebuilds the cmake from 
scratch). You'll need run `catkin_make` again after doing a clean.


# Notes

1. When developing ROS code, Stack Overflow is your best friend. While ChatGPT 
is helpful, it can often be misleading when troubleshooting hardware issues. I
highly reccomend that you stay away from AI code generation tools when doing 
the embedded side of this project, but you may find it useful for the higher 
level code, such as the mission states. 

2. Don't be afraid to ask for help. If you are working on this codebase and have 
questions, please do not hesitate to reach out to Riley Mark, (I wrote most of 
this stuff), and I will help you troubleshoot to the best of my ability.

<b>Riley Mark: rileyjmark@gmail.com</b>


3. Dr. Lawrence is here to help! As scary as he may seem, Dr. Lawrence uses this
project as a way to teach and develop students, and he is here to help you do 
some real design work while in school. Many Rocksat team members before you have 
landed sweet jobs out of school soley because of this project, but it takes a 
lot of sacrifice and effort to make it happen. One of the best ways to be 
successful on this project is to utilize the mentors around you, so I suggest 
keeping Dr. Lawrence in the loop, and get his advice often. (Also, Dr. Lawrence 
lives for sarcasm, so don't be too serious)
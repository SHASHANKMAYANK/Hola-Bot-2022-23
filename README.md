# Hola-Bot-2022-23
Task 0 - 
In task 0 we were asked to install Ubuntu 20.04 and ROS Noetic in our systems
After that objective of the task is to move the turtle inside the turtlesim 324 window in a vertical D shape of radius 1 unit.
2. Procedure

Create package

First, create a workspace, refer link 451 to know how to do so. Once done, compile and source the packages. The file structuring can be understood from link 181.
Cloning the HolA Bot (HB) repository

    Navigate inside your catkin_ws/src directory. If your catkin workspace’s name is catkin_ws & is addressed at home location in your system, use-

cd ~/catkin_ws/src

    Clone the repository…

git clone https://github.com/erts-RnD/eYRC-2022_HolA_Bot

    For users who do not have git installed. Enter the following command for installation-

    sudo apt install git

    HolA Bot package might take some time to clone, depending on your internet speed.
    Install additional packages-

sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
cd ~/catkin_ws

    Finally, build the catkin workspace using catkin_make command. The setup is now complete!

    NOTE: To build the package in the system, ensure that the terminal is pointing at ~/catkin_ws directory and not in ~/catkin_ws/src. Furthermore, once within ~/catkin_ws location, enter catkin_make.

    After the package has been successfully built, do not forget to source it

source devel/setup.bash

    For every new package cloned or created within the src of your catkin workspace, you should build & source the workspace to proceed.

    IMPORTANT NOTE:
    The referred documentation about ROS packages, is in Python 2, but remember, ROS Noetic runs on python 3. You should follow the resources given and implement your solution in Python 3.

    Launch file can run multiple nodes, unlike a python/cpp script, at least in ROS. While in ROS2, this is not the case. Do explore it. Please note that resources related to ROS 2 are not a part of this competition.

To run the task0 launch file, enter:

roslaunch eyrc-2022_holabot task0.launch

    This should run four processes in parallel.
        roscore
        turtlesim_node
        task_0.py
        task0Ex

Implementation

    Now edit the task_0.py file (navigate the clones repo to find where it is) and implement your logic. Do not forget to refer the learning resources section.


**Task 1 **

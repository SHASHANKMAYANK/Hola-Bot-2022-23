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

So let’s get started with Task 1!

Instead of sharing a downloadable (to get a quick start and for the purpose of auto evel) in github as we did in Task 0, we shall take a different approach for now! What’s that you ask?

This time we shall go step by step and do it all ourselves. So we shall create the package from scratch! (For the purpose of practice, but eventually we may share the exact file structure or a downloadable for the purpose of auto-evaluation, more elaboration in upcoming pages.)

    We must already have a catkin workspace to work in.
    Let’s create a package using catkin_create_pkg. The package will just have a dependency on rospy and gazebo.
    Let’s create an hola_bot.urdf file in a subfolder .\src\your_package_name\urdf.
    Let’s also create a hola_bot.launch in a subfolder .\src\your_package_name\launch.
    Finally, let’s also make sure we have installed the teleop_twist_keyboard package that we will be using to manually control the robot model launched in gazebo.
    sudo apt-get install ros-noetic-teleop-twist-keyboard

That’s it! :stuck_out_tongue:
If

the .launch file and and .urdf file was ready, that is:

    .urdf file would have the definition of all the
    links and joints on the robot,
    along with the planar move gazebo plugin to define a model for actuating and feedback.
    .launch file would simply
    launch empty_world.launch and
    run spawn_model node from gazebo_ros package
    with the hola_bot.urdf as an argument.

Then

we would be able to

    run hola_bot.launch to open gazebo with the bot inside it
    and rosrun teleop_twist_keyboard teleop_twist_keyboard.py to control the simulated robot!

But the .launch and .urdf is not ready yet…
Well the .launch file would be so small it is hardly even needed but probably makes things convenient.
So first let’s take a look at what goes into the hola_bot.launch file.
hola_bot.launch

Well as said earlier, this launch file has only 10 lines:

<launch>
  <include
    file="$(find gazebo_ros)/launch/empty_world.launch" />
  <node
    name="spawn_model"
    pkg="gazebo_ros"
    type="spawn_model"
    args="-file $(find your_package_name)/urdf/hola_bot.urdf -urdf -model hola_bot"
    output="screen" />
</launch>

Hope this is self-explanatory. As said it just launches an empty world and spawns the robot model in it!
hola_bot.urdf

Now for the .urdf file,
Following is part of the contents of the urdf file:

<?xml version="1.0" encoding="utf-8"?>
<!--First version of this URDF was automatically created by SolidWorks to URDF Exporter!
    as part of eYSIP 2022 
    Originally created by Stephen Brawner (brawner@gmail.com) 
    Commit Version: 1.6.0-4-g7f85cfe  Build Version: 1.6.7995.38578
    For more information, please see http://wiki.ros.org/sw_urdf_exporter 

    This version of the URDF is made after a lot of simplification by the e-Yantra Team (Arjun Sadananda) -->

<robot
  name="hola_bot">

  <!-- Chassis -->

  <link
    name="base_link">
    <inertial>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <mass
        value="0.49" />
      <inertia
        ixx="0.00199"
        ixy="0"
        ixz="0"
        iyy="0.00199"
        iyz="0"
        izz="0.00395" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 1.57" />
      <geometry>
        <mesh
          filename="package://your_package_name/meshes/base_link.STL" />
      </geometry>
    </visual>
    <collision>
      <origin
        xyz="0 0 -0.01"
        rpy="0 0 1.57" />
      <geometry>
        <cylinder radius="0.15" length="0.005"/>
      </geometry>
    </collision>
  </link>

.
.
<!-- lot more stuff needs to go here -->
.
.
</robot>

This is how we’ll need to start the robot definition.

<?xml version="1.0" encoding="utf-8"?>
This XML declaration just describes some of the most general properties of the document.

<robot name="hola_bot">
Start of Robot Definition
Inside which we shall have all of the remaining definitions, which will end with the last line of the file: </robot>

<link name="base_link">
</link>
This element is the definition of the chassis of the robot. A single rigid body.

    inertial : To define the inertial properties of the rigid body (chassis)
        origin
        mass
        inertia matrix: this was generated by solidworks and may or may not accurate, but do note it is a diagonal matrix as expected.
    visual: To describe only how it LOOKS.
        origin
        geometry: Here we enter an STL file (mesh file) for the purpose of visualisation.
        NOTE: STL file shared below
    collision: To describe its interaction with the world.
        origin
        geometry: Here we simplify the shape to simple cylinders to make processing collisions easier for gazebo (while still maintaining a reasonable match with the real shape).

Similarly here is the definition of one more link: the front wheel of a three-omni-wheeled bot:

<!-- Front Wheel -->

  <!-- Front Wheel Link -->
  <link
    name="front_wheel">
    <inertial>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <mass
        value="0.062" />
      <inertia
        ixx="0.000285"
        ixy="0"
        ixz="0"
        iyy="0.0005198"
        iyz="0"
        izz="0.0002851" />
    </inertial>
    <visual>
      <origin
        xyz="0 0 0"
        rpy="0 0 0" />
      <geometry>
        <mesh
          filename="package://hola_bot/meshes/wheel.STL" />
      </geometry>
    </visual>
    <collision>
      <origin
        xyz="0 -.01 0"
        rpy="1.57 0 0" />
      <geometry>
        <cylinder radius=".03" length="0.02"/>
      </geometry>
    </collision>
  </link>

This is almost exactly the same as the chassis link definition. So with no further explanation, Let’s add this to the above robot definition.

Ah yes, the STL files.
We’ll need the STL files for the chassis and the wheel.
Here are the STL download link:

Download Chassis STL File 465
Download Omni Wheel STL File 419

Next,

Now that we have two links in our file, let’s add a joint between the two.

  <!-- Front Wheel Joint -->

  <joint
    name="front_joint"
    type="continuous">
    <origin
      xyz="0 0.17483 -0.0335"
      rpy="0 0 0" />
    <parent
      link="base_link" />
    <child
      link="front_wheel" />
    <axis
      xyz="0 -1 0" />
    <limit
      effort="5"
      velocity="5" />
  </joint>

<joint name="front_joint" type="continuous">
</joint>

This element is how we define a joint between two links.

the type = “continuous” here means to create a simple revolute joint.

The elements within this element are pretty self-explanatory.

The origin x,y,z, roll, pitch, yaw, is what defines the position of the joint between the “parent link” and the “child link”. The axis define the axis of the revolute joint being defined. And the limit puts some limit on the motion of the joint.

And there you go! we have a robot with one wheel! :stuck_out_tongue:
All we need to do now is add two more wheels to the bot, and of course the gazebo plugin!
Task

Problem Statement: Add two more wheels (left wheel and right wheel) to the urdf file.

One useful hint:
the main element that changes from front wheel to left or right wheel is it’s origin.
The x and y will change to +or- 0.15154 and +or- 0.087275 for the left/right wheels.
I’ll let you figure out the roll, pitch, and yaw by yourself.

Finally to finish the URDF File:

Let’s add the Gazebo Plugin

    <gazebo>
    <plugin name="object_controller" filename="libgazebo_ros_planar_move.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryRate>20.0</odometryRate>
      <robotBaseFrame>base_link</robotBaseFrame>
    </plugin>
  </gazebo>

And voila! That’s actually it!
Just launch the launch file, run the teleop node and have fun driving around HolA Bot!

NOTE: Observe the Holonomic Mode in teleop package. and figure out what teleop is actually doing by echo the rostopic: cmd_vel.

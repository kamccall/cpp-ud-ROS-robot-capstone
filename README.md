# cpp-ud-ROS-robot-capstone
udacity c++ capstone programming assignment using ROS gazebo, teleop, custom nodes and messages to display odometry

## project description
i chose the ROS project, choosing to implement two different nodes, but doing so using many other ROS services in order to learn and understand how all the ROS services work together.  specifically, i started by going through two ROS programming books in order to understand how everything works (*mastering ROS for robotics programming*, third edition, joseph and cacace, copyright 2021, and *hands-on ROS for robotics programming*, japon, copyright 2020). this took 120+ hours, and involved many (normally small) individual programming assignments in order to work through many different concepts.  while this project is arguably a minimal amount of c++ code, i wanted to select a few end-to-end scenarios that combine various important  concepts and services in order to demonstrate a large surface area of what is possible with ROS. 

more specifically, there are three demo tracks that show use of various ROS services:
* diffbot robot shown in `gazebo`, controlled by teleop with custom odometry display
* use of `moveit!` and `navigation` services to create an environment map to support autonomous movement 
* demo of planning and autonomous movement of diffbot through `gazebo` environment controlled by `rviz` based on map created in (2) above 

## file and directory structure
the (more detailed) demo summarized below will refer to the files and how they are used. i have organized the work into two distinct ROS packages, that both exist in the `catkin` workspace hierarchy. both of these ROS packages are therefore placed under the `/home/username/catkin_ws/src` hierarchy, and are therefore accessed most directly by `$ cd ~/catkin_ws/src/package_name`. 
the files and directories under each of the demo solutions are:
### `km_diff_robot_gazebo project` project (`~/catkin_ws/src/km_diff_robot_gazebo`)
* `package.xml` file: ROS `.xml` file that defines all the ROS package dependencies for compilation and execution
* `CMakeLists.txt` file: CMake file that details all package and custom message dependencies but also c++ executables
* `config` dir: ROS `.yaml` files relevant for robot definition
* `launch` dir: ROS `.launch` files for loading robot (into gazebo) and running python teleop application (to explore gazebo environment)
* `msg` dir: custom telemetry message definition that will be published in ROS node
* `scripts` dir: python script to run teleop program
* `src` dir: `.cpp` files for ROS nodes that publish and subscribe various odometry messages (and subsets thereof)
* `urdf` dir: ROS `.xacro` files for diffbot and constituent wheel definition

### `km_diff_robot_gazebo_auto` project (`~/catkin_ws/src/km_diff_robot_gazebo_auto`)
* `package.xml` file: ROS `.xml` file that defines all the ROS package dependencies for compilation and execution
* `CMakeLists.txt` file: CMake file that details all package and custom message dependencies but also c++ executables
* `config` dir: ROS `.yaml` files relevant for SLAM mapping and global and local cost estimates when navigating
* `launch` dir: ROS `.launch` files for loading robot (into gazebo), running mapping algorithm, running python teleop application, and then performing autonomous navigation within the mapped environment using `AMCL`
* `maps` dir: ROS `.pgm` and `.yaml` files that define mapped environment after use of `gmapping` utility
* `scripts` dir: python script to run teleop program
* `urdf` dir: ROS `.xacro` files for diffbot and constituent wheel definition

## installation implications and required packages
while the rubric requests cross-platform installation instructions, given the well-documented differences (and challenges) of having a similar ROS experience on windows as well as the (disproportionate) focus on linux within the community for doing ROS development, i am taking the liberty of only providing install instructions for linux.  oh udacity gods please forgive me for this omission and do not send snakes to my bed because they completely freak me out (did you see the video of that guy who kissed the cobra he saved in new delhi and was bitten in the face? really really poor judgement, dude). 

installation instructions on linux:
1. install ubuntu linux 20.04.5 LTS 
2. install VSCode (from software library in ubuntu)
3. install ROS noetic ninjemys and set up catkin workspace
   - `$ sudo apt update`
   - `$ sudo apt install ros-noetic-desktop-full`
   - `$ sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
   - `$ mkdir -p ~/catkin_ws/src`
   - `$ source /opt/ros/noetic/setup.bash`
   - `$ cd ~/catkin_ws/src`
   - `$ catkin_init_workspace`
   - `$ cd ~/catkin_ws`
   - `$ catkin_make`
5. verify gcc and python compiler versions installed (shouldn't normally be necessary)
   - `$ python3 --version`
   - `$ gcc --version'
7. install (a staggering number of) packages using `$ sudo apt-get install package_name`
   - `ros-noetic-urdf` (might not be necessary)
   - `ros-noetic-xacro` (might not be necessary)
   - `liburdfdom-tools`
   - `ros-noetic-gazebo-ros-pkgs`
   - `ros-noetic-gazebo-msgs`
   - `ros-noetic-gazebo-plugins`
   - `ros-noetic-gazebo-ros-control`
   - `ros-noetic-joy`
   - `ros-noetic-moveit`
   - `ros-noetic-moveit-plugins`
   - `ros-noetic-moveit-planners`
   - `ros-noetic-joint-state-controller`
   - `ros-noetic-position-controllers`
   - `ros-noetic-joint-trajectory-controller`
   - `ros-noetic-navigation`
   - `ros-noetic-gmapping`
   - `ros-noetic-map-server`
8. git clone repo and compile environment
   - `$ git clone https://github.com/kamccall/cpp-ud-ROS-robot-capstone.git`
   - (within cloned repo) `$ cp -r km_diff_robot_gazebo ~/catkin_ws/src`
   - (within cloned repo) `$ cp -r km_diff_robot_gazebo_auto ~/catkin_ws/src`
   - `$ cd ~/catkin_ws`
   - `$ catkin_make`

## demo and behavior
i have organized the two projects into three discrete demonstrations.  the first requires five linux terminal windows and the second and third require four unique windows to perform properly. (this assumes that both packages have been compiled using instructions above.)

### demo1: manually controlling robot in gazebo environment with custom odometry display (each in separate terminal)
1. `$ roscore` (starts ROS master server, parameter server and other services)
2. `$ roslaunch km_diff_robot_gazebo diff_wheeled_gazebo_willow.launch` (launches gazebo with diffbot in willow environment)
3. `$ roslaunch km_diff_robot_gazebo keyboard_teleop.launch` (launches teleop application)
4. `$ rosrun km_diff_robot_gazebo sub_robot_odometry` (launches app that will display forward and angular velocity)
5. `$ rosrun km_diff_robot_gazebo sub_robot_location` (launches app that will display location subset, only x and y coordinates)
6. click into active teleop window running python script, and use keystrokes to move robot, observing movement and node output

### demo2: manually navigating robot through gazebo to create map of environment (to support demo3)


### demo3: autonomously navigate robot through gazebo from rviz using saved environment map



## rubric points addressed




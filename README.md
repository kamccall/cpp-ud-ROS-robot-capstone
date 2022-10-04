# cpp-ud-ROS-robot-capstone
udacity c++ capstone programming assignment using ROS gazebo, teleop, custom nodes and messages to display odometry

## project description
i chose the ROS project, choosing to implement two different nodes, but doing so using many other ROS services in order to learn and understand how all the ROS services work together.  specifically, i started by going through two ROS programming books in order to understand how everything works ('mastering ROS for robotics programming', third edition, joseph and cacace, copyright 2021, and 'hands-on ROS for robotics programming', japon, copyright 2020). this took 120+ hours, and involved many (normally small) individual programming assignments in order to work through many different concepts.  while this project is arguably a minimal amount of c++ code, i wanted to select a few end-to-end scenarios that combine various important  concepts and services in order to demonstrate a large surface area of what is possible with ROS. 

more specifically, there are three demo tracks that show use of various ROS services:
* diffbot robot shown in gazebo, controlled by teleop with custom odometry display
* use of moveit! and navigation services to create an environment map to support autonomous movement 
* demo of planning and autonomous movement of diffbot through gazebo environment controlled by rviz based on map created in (2) above 

## file and directory structure
the (more detailed) demo summarized below will refer to the files and how they are used. i have organized the work into two distinct ROS packages, that both exist in the 'catkin' workspace hierarchy. both of these ROS packages are therefore placed under the /home/username/catkin_ws/src hierarchy, and are therefore accessed most directly by going to ~/catkin_ws/src/package_name. 
the files and directories under each of the demo solutions are:
### km_diff_robot_gazebo project (~/catkin_ws/src/km_diff_robot_gazebo)
* package.xml file: ROS .xml file that defines all the ROS package dependencies for compilation and execution
* CMakeLists.txt file: CMake file that details all package and custom message dependencies but also c++ executables
* config dir: ROS .yaml files relevant for robot definition
* launch dir: ROS .launch files for loading robot (into gazebo) and running python teleop application (to explore gazebo environment)
* msg dir: custom telemetry message that will be published in ROS node
* scripts dir: python script to run teleop program
* src dir: .cpp files for ROS nodes that publish and subscribe various odometry messages (and subsets thereof)
* urdf dir: ROS .xacro files for diffbot and constituent wheel definition

### km_diff_robot_gazebo_auto project (~/catkin_ws/src/km_diff_robot_gazebo_auto)
* package.xml file: ROS file that defines all the ROS package dependencies for compilation and execution
* CMakeLists.txt file: CMake file that details all package and custom message dependencies but also c++ executables
* config dir: ROS .yaml files relevant for SLAM mapping and global and local cost estimates when navigating
* launch dir: ROS .launch files for loading robot (into gazebo), running mapping algorithm, running python teleop application, and then performing autonomous navigation within the mapped environment using AMCL
* maps dir: ROS.pgm and .yaml files that define mapped environment after use of gmapping utility
* scripts dir: python script to run teleop program
* urdf dir: ROS .xacro files for diffbot and constituent wheel definition

## installation implications and required packages
while the rubric requests cross-platform installation instructions, given the well-documented differences (and challenges) of having a similar ROS experience on windows as well as the (disproportionate) focus on linux within the community for doing ROS development, i am taking the liberty of only providing install instructions for linux.  oh udacity gods please forgive me for this omission and do not send snakes to my bed because they completely freak me out (did you see the video of that guy who kissed the cobra he saved in new delhi and was bitten in the face? really really poor judgement, dude). 

installation instructions on linux:
1. install ubuntu linux 20.04.5 LTS 
2. install VSCode (from software library in ubuntu)
3. install ROS noetic ninjemys (full installation)
4. verify gcc and python compiler versions installed (shouldn't normally be necessary)
5. install (a staggering number of) packages using `$ sudo apt get install package_name`
   - ros-noetic-urdf (might not be necessary)
   - ros-noetic-xacro (might not be necessary)
   - liburdfdom-tools
   - ros-noetic-gazebo-ros-pkgs
   - ros-noetic-gazebo-msgs
   - ros-noetic-gazebo-plugins
   - ros-noetic-gazebo-ros-control
   - ros-noetic-joy
   - ros-noetic-moveit
   - ros-noetic-moveit-plugins
   - ros-noetic-moveit-planners
   - ros-noetic-joint-state-controller
   - ros-noetic-position-controllers
   - ros-noetic-joint-trajectory-controller
   - ros-noetic-navigation
   - ros-noetic-gmapping
   - ros-noetic-map-server

## demo and behavior



## rubric points addressed




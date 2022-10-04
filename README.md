# cpp-ud-ROS-robot-capstone
udacity c++ capstone programming assignment using ROS gazebo, teleop, custom nodes and messages to display odometry

## project description
i chose the ROS project, choosing to implement two different nodes, but doing so using many other ROS services in order to learn and understand how all the ROS services work together.  specifically, i started by going through two ROS programming books in order to understand how everything works ('mastering ROS for robotics programming', third edition, joseph and cacace, copyright 2021, and 'hands-on ROS for robotics programming', japon, copyright 2020). this took 120+ hours, and involved many (normally small) individual programming assignments in order to work through many different concepts.  while this project is arguably a minimal amount of c++ code, i wanted to select a few end-to-end scenarios that combine various important  concepts and services in order to demonstrate a large surface area of what is possible with ROS. 

more specifically, there are three demo tracks that show use of various ROS services:
* diffbot robot shown in gazebo, controlled by teleop with custom odometry display
* use of moveit! and navigation services to create an environment map to support autonomous movement 
* demo of planning and autonomous movement of diffbot through gazebo environment controlled by rviz based on map created in (2) above 

## file and directory structure
the (more detailed) demo summarized below will refer to the files and how they are used. i have organized the work into two distinct ROS packages, that both exist in the 'catkin' workspace hierarchy. both of these ROS packages are therefore placed under the /home/username/catkin_ws/src hierarchy, and are therefore accessed most directly by doing to ~/catkin_ws/src/package_name. in this case, both of the solutions exist there. 
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


## demo and behavior


## rubric points addressed




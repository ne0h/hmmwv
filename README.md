HMMWV
=====
HMMWVs your life!

Build Workspace
---------------
1. Install Ubuntu (build with 12.04 LTS)
2. Install ROS Hydro Stuff (ROS Version >= Groovy necessary because of catkin workspace management) as in the [installation manual](http://wiki.ros.org/hydro/Installation/Ubuntu). You need all the basic libs and the tutorial package and libsdl (1.2 should do) and build-essential.
	All of this comes with the meta package ros-hydro-desktop.
3. Clone Repository and source *./src/ros_workspace/devel/setup.bash && /opt/ros/hydro/setup.bash* 
4. Navigate to *./src/ros_workspace* and type *catkin_make*
5. Start up *roscore* and *turtlesim turtlesim_node*
6. Enjoy!

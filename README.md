HMMWV
=====
HMMWVs your life!

Build Workspace
---------------
1. Install Ubuntu (build with 12.04 LTS)
1. Install ROS Hydro Stuff (ROS Version >= Groovy necessary because of catkin workspace management) as in the [installation manual](http://wiki.ros.org/hydro/Installation/Ubuntu). You need all the basic libs, the tutorial package, libsdl1.2-devel and build-essential. All of this except sdl-devel comes with the meta package ros-hydro-desktop.
1. Clone Repository and source *./src/ros_workspace/devel/setup.bash && /opt/ros/hydro/setup.bash* 
1. Navigate to *./src/ros_workspace* and type *catkin_make*
1. Run *rosmake turtlesim*
1. Run *roscore* and *turtlesim.sh*
1. Enjoy!

If you want to run a node from somewhere else than the roscore, you need to set *ROS_MASTER_URI=http://<host>:11311*.
Take a look at *turtlesim.sh* and *hmmwv.sh* for increased convenience.

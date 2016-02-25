HMMWV
=====

Abstract
---------------

HMMWV is a medium sized robot designed to do the following tasks:

* drive by remote control
* navigate automatically
* find and climb steps and stairs

To prepare for this task it has a stiff chassis made from aluminum profiles and two strong y-shaped wheel mounts with 3 individual wheels each which can be tilted individually by two high torque EC maxon motors.

The wheels at the mounts are connected via a chain drive on either side which is powered by a 60 W motor. To prevent the robot from overturning, it has a tail with an omni-wheel that stabilizes the robot if climbing up and down the stairs or other obstacles.

For orientation and path-finding HMMWV has a lidar system from Sick (LMS 100) which sends a two dimensional laser scan to the main processing unit.

HMMWV uses a small x86 computer by Gigabyte as its main computer. Due to the fact that a two dimensional cross section of the environment is not enough to get orientation in a three dimensional room, we put the Sensor on a slow rotating platform, which gives us the opportunity to calculate a three dimensional picture of the robot’s surrounding area.

[Come take a look!](https://github.com/hmmwv-team/hmmwv/tree/master/images)

### Current project status

* The chassis of HMMWV is built 
* the suspensions are installed and the motors are in place
* the mainboard is soldered and connects to the Arduino with the motor controllers
* The robot can be driven by remote control
* The robot can map and navigate in mapped areas more or less reliably (rather less, needs tuning)
 
### The next steps

* tune autonomous movement
* install the rotating platform for the lidar sensor

Build ROS Workspace
---------------
1. Install Ubuntu (build with 14.04 LTS)
1. Install ROS Indigo stuff (ROS Version >= Groovy necessary because of catkin workspace management) as in the [installation manual](http://wiki.ros.org/hydro/Installation/Ubuntu). You need all the basic libs (*ros-indigo-desktop*), *ros-indigo-ros-tutorials*, *libsdl1.2-dev*, *build-essential*, *ros-indigo-tf*, *ros-indigo-tf2*, *ros-indigo-move-base* and *ros-indigo-tf2-geometry-msgs*.
2. Ubuntu 14.04 needs a newer version of PCL. Please install as described here: http://pointclouds.org/downloads/linux.html
1. Clone Repository and source */opt/ros/indigo/setup.bash*
2. Run `git submodules update --init` to clone submodules
1. Navigate to *./ros_workspace* and type *catkin_make*
1. Source *./ros_workspace/devel/setup.bash*
1. Run your favourite nodes
1. Enjoy!

If you want to run a node from somewhere else than the roscore, you need to set *ROS_MASTER_URI=http://host:11311*.
Take a look at *turtlesim.sh* and *hmmwv.sh* for increased convenience.

Startup Instructions
--------------------

1. Export a fitting ROS_MASTER_URI on all machines, f.e. `export ROS_MASTER_URI=http://hmmwv:11311`
1. Consider setting ROS_IP or ROS_HOSTNAME as needed for the nodes to find each other.
1. On hmmwv (the mini PC below that wood plate)
	1. `roslaunch hmmwv hmmwv.launch`
	1. `roslaunch hmmwv engine.launch` (Executing user must be member of the `dialout` group.)
		1. This node might be included in the hmmwv.launch file. Use at own risk.
1. On the computer that has the controller plugged in
	1. `roslaunch hmmwv remote.launch`

*Note:* It is recommended to either manually start a roscore on hmmwv first or let hmmwv.launch start one for you _on that computer_.

*Note 2:* This assumes you have a working ros installation and built our robot code successfully.

Example setup
-------------

### hmmwv

* hardware: Mini PC Gigabyte GB-BXCE-2955
* tasks: WLAN access point, navigation
* ros nodes: roscore, lms100, slam_gmapping, enginecontrol, some TFs
* `ROS_MASTER_URI=http://hmmwv:11311` and `ROS_HOSTNAME=hmmwv`

### Arduino (engine controller)

* Arduino Mega 2560
* Accessible via */dev/ttyACM0*, user has to be in group *dialout*
* Eclipse workspace in */enginecontrol*, you need to run *setup_enginecontrol.sh* to get dependencies

Mapping/Odometry
----------------

A few extra nodes apart from the aforementioned lms100 are needed to make ros build a map from the LMS100 sensor data. To put the laser scan data into perspective, ros also needs odometry data provided by us. (Having a laser scan doesn't help much on its own, we also need to know if the obstacle moved or we just drove towards it...)

To see the laser scan data in rviz:
* Start up all that mapping stuff: `roslaunch hmmwv hmmwv.launch`
* Now start rviz. In rviz:
  * Via the "add" button (bottom left), add a "LaserScan" module, set the topic to "lms1xx/scan". Now the laser measurements should be visible in the 3D view. (The orthographic camera is useful...)
  * The "TF" module can help analyzing problems with reference frames.

To enable automatic map generation:
* Make sure you have started up the mapping stuff (see above)
* Start the enginecontrol node (provides odometry data): `roslaunch hmmwv engine.launch`
* Add the "Map" module to rviz. That should show an occupancy grid depicting current map state.
  * The map is updated as the robot moves, *not* regularly in time.

Notes
-----
* The package *hector_filetrain* (hector_mapping) is directly taken from *https://github.com/tu-darmstadt-ros-pkg/hector_slam* to add a transform delay
* The code for the lms100 driver is taken from *https://github.com/ipa320/cob_driver/tree/indigo_dev/cob_sick_lms1xx*
* [Beaglebone Interactive Pin Map](http://eskimon.fr/beaglebone-black-gpio-interactive-map)
* At normal (0.25x) driving speed, the robot covers 2 meters within 6,872 seconds => 1,642 m/s at full speed (1.0x)
* Likewise, on-the-spot rotation speed: 2 revolutions in 29s (0.25x speed) => 1,732 rad/s at full speed (1.0x)

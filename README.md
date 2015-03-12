HMMWV
=====

Abstract
---------------

HMMWV is a medium sized Robot designed to do the following tasks:

* drive by remote control
* drive automatically 
* find and climb steps and stairs
	
To get up with this tasks it has a stiff chassis made with aluminum profiles and two stable wheel suspensions which can be tilted individually by two high torque EC maxon Motors.

The wheels at the suspensions are connected via two chain drives (one on both sides)  which are powered by two 60 W motors. To prevent the robot from overturning, it has a tail with an omni-wheel that stabilizes the robot, if climbing up and down the stairs or other barriers.

For orientation and path-finding HMMWV has a lidar system from Sick (LMS 100) which sends a two dimensional profile to the main processing unit.

As CPU HMMWV uses an ARM based singe-board computer from Texas Instruments (Beaglebone Black). Due to the fact that a two dimensional cross section of the environment is not enough to get orientation in a three dimensional room, we put the Sensor on a slow rotating platform, which gives us the opportunity to calculate an three dimensional picture of the robot’s surrounding area.

If you want to see images please click on the images folder or the following link: https://github.com/hmmwv-team/hmmwv/tree/master/images

### Actual project status

* The chassis of HMMWV is built 
* the suspensions are installed and the motors are in place
* the mainboard is soldered and connects the Beaglebone with the motor controllers
* We are currently on to program the system to move the robot by remote control
 
### The next steps

* we will install the rotating platform for the lidar sensor 
* we will add some code for automatically movement

Build Workspace
---------------
1. Install Ubuntu (build with 12.04 LTS)
1. Install ROS Hydro Stuff (ROS Version >= Groovy necessary because of catkin workspace management) as in the [installation manual](http://wiki.ros.org/hydro/Installation/Ubuntu). You need all the basic libs, the tutorial package, libsdl1.2-devel and build-essential. All of this except sdl-devel comes with the meta package ros-hydro-desktop.
1. Clone Repository and source *./src/ros_workspace/devel/setup.bash && /opt/ros/hydro/setup.bash* 
1. Navigate to *./src/ros_workspace* and type *catkin_make*
1. Run *rosmake turtlesim*
1. Run *roscore* and *turtlesim.sh*
1. Enjoy!

If you want to run a node from somewhere else than the roscore, you need to set *ROS_MASTER_URI=http://host:11311*.
Take a look at *turtlesim.sh* and *hmmwv.sh* for increased convenience.

Startup Instructions for non-techies
------------------------------------

1. SSH into the beagle
1. cd hmmwv
1. roscore &
1. ./remote.sh &
1. sudo su
1. ./engine.sh

BBB GPIO Zen
------------

In the following, $pin refers to pin numbers as used by the kernel. Mappings for the pins we use are available in the `Pin` enum in gpio.h.

Each used pin must be exported before first usage and it's direction needs to be set (in/out):
```
echo $pin > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio/$pin/direction
```
Now the pin value (binary) can be set arbitrarily:
```
echo 1 > /sys/class/gpio/gpio/$pin/value
echo 0 > /sys/class/gpio/gpio/$pin/value
```

Example setup
-------------

### hmmwv

* hardware: Mini PC Gigabyte GB-BXCE-2955
* tasks: WLAN access point, navigation
* ros nodes: roscore, lms100
* `ROS_MASTER_URI=http://hmmwv:11311` and `ROS_HOSTNAME=hmmwv`

### bone

* hardware: beaglebone black
* tasks: engine control
* ros nodes: enginecontrol
* `ROS_MASTER_URI=http://hmmwv:11311` and `ROS_HOSTNAME=bone`

Notes
-----

* The code for the lms100 driver is taken from *https://github.com/ipa320/cob_driver/tree/indigo_dev/cob_sick_lms1xx*
* [Beaglebone Interactive Pin Map](http://eskimon.fr/beaglebone-black-gpio-interactive-map)

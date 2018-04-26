To set up the ROS environment:
1) Run 'catkin_make' in the ROS_RoboBuggy directory
2) Run 'source devel/setup.bash'

This will allow you to use utilities such as roscd, rosrun, roslaunch, etc. 

To rebuild the project after making changes, run 'catkin_make' from within the
ROS_RoboBuggy directory

It might be useful to put a command in your .bashrc file to source the setup script automatically.

To do a clean build, delete the build/ and devel/ folders, and re-run catkin_make

Installing third-party libraries:
1) libfreespace - for the IMU
	- To install libfreespace, follow the instructions at https://github.com/hcrest/libfreespace

Setting up udev rules for the IMU to load as a sudo-readable device by default:
http://shukra.cedt.iisc.ernet.in/edwiki/EmSys:Accessing_Devices_without_Sudo

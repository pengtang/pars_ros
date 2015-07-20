# pars_ros
Incorporating AMCL in ROS to PARS

Instruction on Installation of RosAMCL related stuff

How to install ROS, stdr simulator and AMCL

Follow the instruction on http://wiki.ros.org/indigo/Installation/Ubuntu
Follow the instruction on http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment, create a directory catkin_ws under home directory. And remember to append “source ~/catkin_ws/devel/setup.bash” to the .bashrc file.
Go to ~/catkin_ws/src, git clone https://github.com/pengtang/pars_ros.git. git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git
Copy the star.launch and star_localization.launch file to the directory of ~/catkin_ws/src/stdr_simulator/stdr_server. Copy the laser_tf_broadcaster.cpp to ~/catkin_ws/src/stdr_simulator/stdr_server/src. copy amcl.h to the directory of pars
sudo apt-get install ros-indigo-amcl
Go to ~/catkin_ws run catkin_make
Replace a few files in pars. git  clone https://github.com/pengtang/pars_1.5.git. 
Edit the star.launch in ~/catkin_ws/src/stdr_simulator/stdr_server (change the map location)
In pars directory, make clean and then make pars


How to boot up.

Way 1. Manually boot up. Fire up 4 terminal tabs. The first one runs “roscore", the second one runs "rosrun map_server map_server catkin_ws/src/stdr_simulator/stdr_resources/maps/map.yaml” (replace the last argument with your map location). The third one runs "roslaunch stdr_server stdr.launch”. The fourth one runs "roslaunch stdr_server stdr_localization.launch"


NOTE: remove /tmp/myfifo before rerun another mission

Way 2. Using the constructor with input argument of the map.
	— Edit the function rosamcl in dlibinterface.cpp, comment the "AMCL a;” and uncomment   //AMCL a("/home/pengtang/catkin_ws/src/stdr_simulator/stdr_resources/maps/map.yaml");
  //extern AMCL a; Replace the input argument with the location of the map.

NOTE: Please kill all the related processes and remove /tmp/myfifo before rerun another mission. These processes are still running even when vipars finishes running

Run the mission.

After booting up the ROS related stuff, just run the mission

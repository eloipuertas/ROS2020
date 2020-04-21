# ROS

## Session 1

### Virtual Box Installation guide:

First Download Ubuntu 16.04.4 LTS ISO
Then install VirtualBox and add a new Ubuntu 64bits Machine with a mínimum of Virtual HD 20GB space and 8GB RAM Then Start the machine and select the Ubuntu ISO file. Follow all the steps for installing Ubunto in your Virtual Box. (follow [this](https://itsfoss.com/install-linux-in-virtualbox/) guide if you are in trouble)

* [Ubuntu 16.04.4 LTS Xenial Xerus Desktop image 64 Bit](http://releases.ubuntu.com/16.04/)
* [VirtualBox](https://www.virtualbox.org/)
* For shared clipboard, go to Devices and select shared Clipboard bidirectional. If it doesn't work, select Insert Guest Additions CD Image in the same menu. **(There's a bug in the 6.1.4 version, use the 6.1.5 from [here](https://www.virtualbox.org/download/testcase/VBoxGuestAdditions_6.1.5-136807.iso) instead)**


### ROS Installation guide
Now Run your new Ubuntu Virtual Box machine and install the ROS system following the next guide:

* [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
	* Chose Desktop-Full Install distribution.
* [Installing and Configuring ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
	* Create a ROS Workspace using catkin. We are gone use Python 2.7 for our robot for compatibility reasons, thus don't follow **Python 3 users** steps.

### Ros Overview

* [ROS Overview](http://wiki.ros.org/ROS/Introduction)

# Creating your first project
* [Create your first Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
* [Building your first Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)



# GAZEBO
* [Install Gazebo 7.x and Integration with ROS using gazebo_ros_pkgs ROS-Kinetic pre-builts  ](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

Turtelebo3 Gazebo Package:
* [Install dependent ROS packages](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-1-packages)

## Session 2

# ROBOTS
* [Turtlebot3 burger simulation using gazebo](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
* [URDF robot examples](https://wiki.ros.org/urdf/Examples)



# Schedule

|  Date |  Lesson |   Goal|   
|--- | --- | ---|
|  14/04/2020 | Introduction to ROS  | Install ROS and Gazebo
|  21/04/2020|  Using Turtlebot with Gazebo|  Follow the tutorial about Turtle Bot simulation using gazebo.  
|  28/04/2020 | Publisher and Subscribers | Create 1st project in ROS   
|  5/05/2020  | OpenCV and ROS| Project ROS using OpenCV   
|  12/04/2020 | SLAM and ROS| Project ROS using SLAM   
|  19/04/2020 | Exiting the Maze | Exiting Maze Final Project  
|  26/04/2020 | Final Project Presentation | Final Project Presentation

# ROS

## Session 1

### Virtual Box Installation guide:

First Download Ubuntu 16.04.4 LTS ISO
Then install VirtualBox and add a new Ubuntu 64bits Machine with a mínimum of Virtual HD 20GB space and 8GB RAM Then Start the machine and select the Ubuntu ISO file. Follow all the steps for installing Ubuntu in your Virtual Box. (follow [this](https://itsfoss.com/install-linux-in-virtualbox/) guide if you are in trouble)

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

### Creating your first project
* [Create your first Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
* [Building your first Package](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)



### GAZEBO
* [Install Gazebo 7.x and Integration with ROS using gazebo_ros_pkgs ROS-Kinetic pre-builts  ](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)
* [Install dependent ROS packages for TurtleBot](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-1-packages)


## Session 2

### ROBOTS
* [Turtlebot3 burger simulation using gazebo](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
* [URDF robot examples](https://wiki.ros.org/urdf/

## Session 3

### Quick Overview of Graph Concepts
* Nodes: A node is an executable that uses ROS to communicate with other nodes.

* Messages: ROS data type used when subscribing or publishing to a topic.

* Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.

* Master: Name service for ROS (i.e. helps nodes find each other)

* rosout: ROS equivalent of stdout/stderr

* roscore: Master + rosout + parameter server (parameter server will be introduced later)

### Using rosnode
Launch Gazebo with the turtleboot in an empty world. 

```
   $ export TURTLEBOT3_MODEL=burger
   $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

Open up a new terminal, and let's use rosnode to see what is running ... Bear in mind to keep the previous terminal open either by opening a new tab or simply minimizing it.
Rosnode displays information about the ROS nodes that are currently running. The rosnode list command lists these active nodes:

```
$ rosnode list
```

The `rosnode info` command returns information about a specific node.

```
$  rosnode info /gazebo
```

Let's use another rosnode command, ping, to test that it's up:

```
$ rosnode ping gazebo
```

### Understanding ROS Topics

Launch the teleop of our turtlebot in a new terminal: 

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```
The gazebo and the teleop node are communicating with each other over a ROS Topic.
`rqt_graph` creates a dynamic graph of what's going on in the system.

In a new terminal:

```
 rosrun rqt_graph rqt_graph
```
you will see the topic that turtlebot is using to communicate the speed of the robot to gazebo.

The `rostopic` tool allows you to get information about ROS topics.

You can use the help option to get the available sub-commands for `rostopic`

```
rostopic -h
```

Let's use some of these topic sub-commands to examine gaezebo

`rostopic echo` shows the data published on a topic.

```
rostopic echo /cmd_vel
```

You probably won't see anything happen because no data is being published on the topic. Let's make teleop publish data by pressing the arrow keys.

Now let's look at `rqt_graph` again. Press the refresh button in the upper-left to show the new node. As you can see rostopic echo, shown here in red, is now also subscribed to the /cmd_vel topic.

`rostopic list` returns a list of all topics currently subscribed to and published.

Let's figure out what argument the list sub-command needs. In a new terminal run:

```
rostopic list -h
```

This will display a verbose list of topics to publish to and subscribe to and their type:

```
rostopic list -v
```

### ROS Messages

Communication on topics happens by sending ROS messages between nodes. For the publisher (turtlebot3_teleop_key) and subscriber (gazebo) to communicate, the publisher and subscriber must send and receive the same type of message. This means that a topic type is defined by the message type published on it. The type of the message sent on a topic can be determined using `rostopic type`.

`rostopic type` returns the message type of any topic being published. Try

```
rostopic type /cmd_vel

```

We can look at the details of the message using `rosmsg`:

```
rosmsg show geometry_msgs/Twist
```

```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

### Using rostopic pub
`rostopic pub` publishes data on to a topic currently advertised.

```
rostopic pub [topic] [msg_type] [args]
```
Stop teleop node and try:

```
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.22, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
The option dash-one causes rostopic to only publish one message then exit.

The option double-dash tells the option parser that none of the following arguments is an option. This is required in cases where your arguments have a leading dash -, like negative numbers.

You may have noticed that the pub commands ends after a while. We can publish a steady stream of commands using rostopic pub -r command:
```
rostopic pub /cmd_vel geometry_msgs/Twist -r 1 -- '[0.22, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

We can also look at what is happening in rqt_graph. Press the refresh button in the upper-left. The `rostopic pub node`  is communicating with the `rostopic echo node`

### Writing Publisher and Subscriber (Python)
[Writing Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
### Runing Publisher and Subscriber
[Runing Publisher and Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

### Exercises
1. Create a new node to move your robot ahead during a fixed period of time. (Publisher)
2. Create a new node to move your robot ahead until the laser scan detect some object at a fixed distance. (Publisher + Subscriber)




### Tips and help
* Check the Publications and Subscriptions of your robot and the type of messages it returns.
* All the information about the turtlebot topics:
[Turtlebot3](http://wiki.ros.org/turtlebot3_example)
* [How to read laserscan-data](https://www.theconstructsim.com/read-laserscan-data/)
* If you have troubles using your virtual machine, you can use this robot platform in the cloud [ros-development-studio](https://www.theconstructsim.com/rds-ros-development-studio/). Free up to 8h per day.
* The construct. Every Tuesday @ 6 PM CET Live Class. [28 April. Finally understand ROS subscrivers](https://theconstructsim.us12.list-manage.com/track/click?u=3d07cceeeb3e64e1676c8971f&id=96badbd484&e=4889793259)
* All The construct [youtube videos](https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q/videos)

# Schedule

|  Date |  Lesson |   Goal|   
|--- | --- | ---|
|  14/04/2020 | Introduction to ROS  | Install ROS and Gazebo
|  21/04/2020 |  Using Turtlebot with Gazebo|  Follow the tutorial about Turtle Bot simulation using gazebo.  
|  28/04/2020 | Publisher and Subscribers | Create 1st project in ROS     
|  05/04/2020 | SLAM and ROS| Project ROS using SLAM  
|  12/05/2020 | OpenCV and ROS| Project ROS using OpenCV 
|  19/04/2020 | Exiting the Maze | Exiting Maze Final Project  
|  26/04/2020 | Final Project Presentation | Final Project Presentation

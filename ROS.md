<a name="top"></a>
# ROS Course
[Session 1](#Session1) | [Session 2](#Session2) | [Session 3](#Session3) | [Session 4](#Session4) | [Schedule](#Schedule)


<a name="Session1"></a>
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



### GAZEBO simulator
* [Install Gazebo 7.x and Integration with ROS using gazebo_ros_pkgs ROS-Kinetic pre-builts  ](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)

### Ros Development Studio cloud platform.
If you have any trouble using the virtual machine in your computer, you can use this robotic platform in the cloud [ros-development-studio](https://www.theconstructsim.com/rds-ros-development-studio/). Free use up to 8h per day.

### Edit your bash to simplify your RUN
```
   $ gedit ~/.bashrc
```
Then go to final file and add these lines:
```
   source /opt/ros/kinetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   export TURTLEBOT3_MODEL=burger
```
And save it. Finally reopen your shell.

### ROS DEVELOPMENT STUDIO
If you use ros development studio you can follow this tutorial.  
1. Create your project
2. Open a shell:
```
   $ cd ~/catkin_ws/src/
```
3. Do these repositories clones:
```
   git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
   git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
   git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
```
4. Build
```
cd ~/catkin_ws && catkin_make
```
Warning!! For every time that you open a new shell you must write: export TURTLEBOT3_MODEL=burger  
You can edit shell with vim because in ROS online not has gedit.  

<a name="Session2"></a>
## Session 2

### Runing Turtlebo3 in gazebo
* [Install dependent ROS packages for TurtleBot](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-1-packages)
* [Turtlebot3 simulation using gazebo](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) Use `export TURTLEBOT3_MODEL=burger` as tutlebot model.
* [URDF robot examples](https://wiki.ros.org/urdf/)

<a name="Session3"></a>
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
* The construct. Every Tuesday @ 6 PM CET Live Class. [28 April. Finally understand ROS subscribers](https://theconstructsim.us12.list-manage.com/track/click?u=3d07cceeeb3e64e1676c8971f&id=96badbd484&e=4889793259)
* All The construct [youtube videos](https://www.youtube.com/channel/UCt6Lag-vv25fTX3e11mVY1Q/videos)

<a name="Session4"></a>
## Session 4
### Navigation Stack.

The idea behind Navigation Stack is to have a robot that can move in a world without crashing into obstacles, drawing a map for this world or following a predefined map, send locations to the robot to go there and collect information about this world from the robot. For this to happen, Navigation stack collects different types of information from several topics and channels.

1. Sensor Information.
The robot uses the sensor\_msgs/LaserScan or sensor_msgs/PointCloud messgaes from the sensors to communicate with ROS.
2. Odometry Information.
The odometry information’s messages nav_msgs/Odometry are published using tf.
3. Base Controller. 
The navigation stack uses the geometry_msgs/Twist messages to control the velocity of the robot over the topic cmd_vel that is able to take the vx, vy, vtheta –> cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z velocities and convert them to motor commands sent to the mobile base.

![Navigation Stack](nav_stack_layout.png)


### Steps for navigating
To navigate the environment, TurtleBot needs a map, a localization module, and
a path planning module. TurtleBot can safely and autonomously navigate the
environment if the map completely and accurately defines the environment.

1. Generate a map using [gmapping](http://wiki.ros.org/gmapping/) from ladar data. Remember the execution procedure was:
	* Launch Gazebo
	* Launch SLAM
		` roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping `
	* Move the robot arround
	* Save the Map
		`rosrun map_server map_saver -f ~/map ` 
	* Before we begin with the steps for autonomous navigation, check the location of
your .yaml and .pgm map files created.

2. Estimate Initial Pose.
	* Once you have the map, launch the navigation stack in the turtlebot: `roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml`
	If you check the rviz windows, you can notice that TurtleBot does not know its
current location on the map. It needs a little help. Let TurtleBot know this location by performing the following steps:
		*  Click on the 2D Pose Estimate button on the tool toolbar at the top of the riz
main screen.
		* Click on the location on the map in the gazebo simulator where TurtleBot is located and drag the
mouse in the direction TurtleBot is facing in Rviz aplication.
	* **Amcl** is a probabilistic localization system for a robot moving in 2D. It implements the adaptive (or KLD-sampling) Monte Carlo localization approach (as described by Dieter Fox), which uses a particle filter to track the pose of a robot against a known map. The amcl algorithm works to figure out where the robot would need to
be on the map in order for its laser scans to make sense. Each possible location
is represented by a particle. Particles with laser scans that do not match well are
removed, resulting in a group of particles representing the location of the robot
in the map. The amcl node uses the particle positions to compute and publish the
transform from map to base_link. You can know where is your robot subscribing the topic `amcl_pose` 
```rostopic echo /amcl_pose```
[http://wiki.ros.org/amcl](http://wiki.ros.org/amcl)

3. Send Navigation Goal. 
	* Next, we can command TurtleBot to a new location and orientation in the room by
identifying a goal using Rviz:
		* Click on the 2D Nav Goal button on the tool toolbar at the top of the
main screen.
		* Click on the location on the map where you want TurtleBot to go and drag the mouse in the direction TurtleBot should be facing when it is finished.
	* Alternatively you can do it programatically, using the `actionlib package`, creating a `SimpleActionClient` of a `move_base` action type and wait for server. The ROS navigation stack is based on ROS Action,  indeed Actions are the best choice for cases when a node wants to send a request to another node and will receive a response after a relatively long time. To avoid leaving the user wondering what’s happening and if all is going as desired, Actions implement a feedback mechanism, which let the user receive information every now and then. Actions are Client-Server-based: the actionlib library provides the tools and interface to set up an Action Server to execute the requested goals sent by the Client. The main elements of an action mechanisms are: goal, result, and feedback. Each one of them is specified by a ROS Message type, contained in an action definition file, with “.action” extension.
	* The **move\_base**  node implements a `SimpleActionServer`, an action server with a single goal policy, taking in goals of `geometry_msgs/PoseStamped` message type. To communicate with this node, the `SimpleActionClient` interface is used. The `move_base` node tries to achieve a desired pose by combining a global and a local motion planners to accomplish a navigation task which includes obstacle avoidance:
		* Global planner: These processes perform path planning for a robot to reach a
goal on the map.
		* Local planner: These processes perform path planning for a robot to create paths
to nearby locations on a map and avoid obstacles.
		* Global costmap: This costmap keeps information for global navigation. Global
costmap parameters control the global navigation behavior. These parameters are
stored in global\_costmap\_params.yaml. Parameters common to global and local
costmaps are stored in costmap\_common\_params.yaml.
		* Local costmap: This costmap keeps information for local navigation. Local
costmap parameters control the local navigation behavior and are stored in
local\_costmap\_params.yaml.
	[http://wiki.ros.org/move_base](http://wiki.ros.org/move_base)



### Exercise 

* Load the house scenario. 
* Generate the house mapping using multiple robots if you want as explained in the [robotis emanual](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#virtual-slam-by-multiple-turtlebot3s) from [session 2](#Session2).
* Locate the initial position of the robot using Rviz and find the coordinates for the mailbox situated outside of the house. Set as a goal the mailbox door and navigate to there using an actionlib client write in python.  

### Tips and helps
* [Tutorial Map-Based Navigation](https://edu.gaitech.hk/turtlebot/map-navigation.html)
* [http://wiki.ros.org/navigation/Tutorials/RobotSetup#Running_the_Navigation_Stack](http://wiki.ros.org/navigation/Tutorials/RobotSetup#Running_the_Navigation_Stack)
* [ROS Robotics by example](http://dl.booktolearn.com/ebooks2/engineering/robotics/9781782175193_ros_robotics_by_example_8f38.pdf)
* [Sending Goals to the Navigation Stack](https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/)

<a name="Schedule"></a>
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

[Go to Top](#top)

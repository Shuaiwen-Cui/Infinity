# ROS2-Humble Guide

[🌐 Link to the original page](https://docs.ros.org/en/humble/index.html)

## 1. Installation 
Last Updated: Jan 03, 2024

### Ubuntu
Table of Contents

#### Resources
#### Set locale
Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
#### Setup Sources
You will need to add the ROS 2 apt repository to your system.

First ensure that the Ubuntu Universe repository is enabled.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Install ROS 2 packages

Update your apt repository caches after setting up the repositories.

```bash
sudo apt update
```
ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

```bash
sudo apt upgrade
```

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

```bash
sudo apt install ros-humble-desktop
```
ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.

```bash
sudo apt install ros-humble-ros-base
```
Development tools: Compilers and other tools to build ROS packages

```bash
sudo apt install ros-dev-tools
```
#### Environment setup

**Sourcing the setup script**

Set up your environment by sourcing the following file.

```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

If you want to automatically source this script you can add it to your bash session.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
#### Try some examples

**Talker-listener**

If you installed ros-humble-desktop above you can try some examples.

In one terminal, source the setup file and then run a C++ talker:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a Python listener:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```
You should see the talker saying that it is Publishing messages and the listener saying I heard those messages.

#### Next steps after installing
#### Using the ROS 1 bridge
#### Additional RMW implementations (optional)
#### Troubleshooting
#### Uninstall
If you need to uninstall ROS 2 or switch to a source-based install once you have already installed from binaries, run the following command:

```bash
sudo apt remove ~nros-humble-* && sudo apt autoremove
```
You may also want to remove the repository:

```bash
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
```
### Windows
skipped
### RHEL
skipped
### Alternatives
skipped
A list of alternative ways to install ROS 2 – whether it’s by building from source or installing a binary.

### Maintaining Source Checkout
If you installled ROS 2 from source, you can update your source checkout according to this section. Please check the original link.

skipped
### Testing with Pre-release Binaries
skipped

### DDS Implementations

By default, ROS 2 uses DDS as its middleware. It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors. There is currently support for eProsima’s Fast DDS, RTI’s Connext DDS, Eclipse Cyclone DDS, and GurumNetworks GurumDDS. See https://ros.org/reps/rep-2000.html for supported DDS vendors by distribution.

## 2. Distributions
skipped

## 3. Tutorials

### Beginner: CLI Tools
#### Configuring environment
Goal: This tutorial will show you how to prepare your ROS 2 environment.
##### Background
ROS 2 relies on the notion of combining workspaces using the shell environment. “Workspace” is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays. When developing with ROS 2, you will typically have several workspaces active concurrently.

Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier. It also allows the installation of several ROS 2 distributions (or “distros”, e.g. Dashing and Eloquent) on the same computer and switching between them.

This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages. In other words, you won’t be able to use ROS 2.

##### Prerequisites
Before starting these tutorials, install ROS 2 by following the instructions on the ROS 2 Installation page.

The commands used in this tutorial assume you followed the binary packages installation guide for your operating system (Debian packages for Linux). You can still follow along if you built from source, but the path to your setup files will likely be different. You also won’t be able to use the sudo apt install ros-<distro>-<package> command (used frequently in the beginner level tutorials) if you install from source.

If you are using Linux or macOS, but are not already familiar with the shell, this tutorial will help.

##### Tasks

> Source the setup file

You will need to run this command on every new shell you open to have access to the ROS 2 commands, like so:

=== "Linux"

    ```bash
    # Replace ".bash" with your shell if you're not using bash
    # Possible values are: setup.bash, setup.sh, setup.zsh
    source /opt/ros/humble/setup.bash
    ```
=== "macOS"

    ```bash
    . ~/ros2_install/ros2-osx/setup.bash
    ```
=== "Windows"

    ```bash
    call C:\dev\ros2\local_setup.bat
    ```

!!! note
    The exact command depends on where you installed ROS 2. If you’re having problems, ensure the file path leads to your installation.

> Add sourcing to your shell startup script

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

=== "Linux"

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
=== "macOS"

    ```bash
    echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile
    ```
=== "Windows"

    Only for PowerShell users, create a folder in ‘My Documents’ called ‘WindowsPowerShell’. Within ‘WindowsPowerShell’, create file ‘Microsoft.PowerShell_profile.ps1’. Inside the file, paste:

    ```bash
    C:\dev\ros2_humble\local_setup.ps1
    ```

    PowerShell will request permission to run this script everytime a new shell is opened. To avoid that issue you can run:

    ```bash
    Unblock-File C:\dev\ros2_humble\local_setup.ps1
    ```

    To undo this, remove the new ‘Microsoft.PowerShell_profile.ps1’ file.

> Check Environment Variables

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2. If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly set up using the following command:

=== "Linux"

    ```bash
    printenv | grep ROS
    ```

=== "macOS"

    ```bash
    printenv | grep -i ROS
    ```

=== "Windows"

    ```bash
    set | findstr -i ROS
    ```

Check that variables like ROS_DISTRO and ROS_VERSION are set.

```bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```
If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed. 

[The ROS_DOMAIN_ID variable]

See the domain ID article for details on ROS domain IDs.

Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command:

=== "Linux"

    ```bash
    export ROS_DOMAIN_ID=<your_domain_id>
    ```

    To maintain this setting between shell sessions, you can add the command to your shell startup script:

    ```bash
    echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
    ```

=== "macOS"

    ```bash
    export ROS_DOMAIN_ID=<your_domain_id>
    ```

    To maintain this setting between shell sessions, you can add the command to your shell startup script:

    ```bash
    echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bash_profile
    ```
=== "Windows"

    ```bash
    set ROS_DOMAIN_ID=<your_domain_id>
    ```
    If you want to make this permanent between shell sessions, also run:

    ```bash
    setx ROS_DOMAIN_ID <your_domain_id>
    ```

[3.2 The ROS_LOCALHOST_ONLY variable]

By default, ROS 2 communication is not limited to localhost. ROS_LOCALHOST_ONLY environment variable allows you to limit ROS 2 communication to localhost only. This means your ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network. Using ROS_LOCALHOST_ONLY is helpful in certain settings, such as classrooms, where multiple robots may publish to the same topic causing strange behaviors. You can set the environment variable with the following command:

=== "Linux"

    ```bash
    export ROS_LOCALHOST_ONLY=1
    ```

    To maintain this setting between shell sessions, you can add the command to your shell startup script:

    ```bash
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
    ```
=== "macOS"

    ```bash
    export ROS_LOCALHOST_ONLY=1
    ```
    To maintain this setting between shell sessions, you can add the command to your shell startup script:

    ```bash
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bash_profile
    ```
=== "Windows"

    ```bash
    set ROS_LOCALHOST_ONLY=1
    ```
    If you want to make this permanent between shell sessions, also run:

    ```bash
    setx ROS_LOCALHOST_ONLY 1
    ```
##### Summary

The ROS 2 development environment needs to be correctly configured before use. This can be done in two ways: either sourcing the setup files in every new shell you open, or adding the source command to your startup script.

If you ever face any problems locating or using packages with ROS 2, the first thing you should do is check your environment variables and ensure they are set to the version and distro you intended.

#### Using turtlesim, ros2, and rqt
Goal: Install and use the turtlesim package and rqt tools to prepare for upcoming tutorials.

##### Background
Turtlesim is a lightweight simulator for learning ROS 2. It illustrates what ROS 2 does at the most basic level to give you an idea of what you will do with a real robot or a robot simulation later on.

The ros2 tool is how the user manages, introspects, and interacts with a ROS system. It supports multiple commands that target different aspects of the system and its operation. One might use it to start a node, set a parameter, listen to a topic, and many more. The ros2 tool is part of the core ROS 2 installation.

rqt is a graphical user interface (GUI) tool for ROS 2. Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.

This tutorial touches upon core ROS 2 concepts, like nodes, topics, and services. All of these concepts will be elaborated on in later tutorials; for now, you will simply set up the tools and get a feel for them.

##### Prerequisites
~

##### Tasks

> 1 Install turtlesim

As always, start by sourcing your setup files in a new terminal, as described in the previous tutorial.

Install the turtlesim package for your ROS 2 distro:

=== "Linux"

    ```bash
    sudo apt update

    sudo apt install ros-humble-turtlesim
    ```
=== "macOS"

    As long as the archive you installed ROS 2 from contains the ros_tutorials repository, you should already have turtlesim installed.

=== "Windows"

    As long as the archive you installed ROS 2 from contains the ros_tutorials repository, you should already have turtlesim installed.

Check that the package is installed:

```bash
ros2 pkg executables turtlesim
```
The above command should return a list of turtlesim’s executables:

```bash
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```
> 2 Start turtlesim

To start turtlesim, enter the following command in your terminal:

```bash
ros2 run turtlesim turtlesim_node
```
The simulator window should appear, with a random turtle in the center.

In the terminal, under the command, you will see messages from the node:

```bash
[INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```
> 3 Use turtlesim

Open a new terminal and source ROS 2 again.

Now you will run a new node to control the turtle in the first node:

```bash
ros2 run turtlesim turtle_teleop_key
```
At this point you should have three windows open: a terminal running turtlesim_node, a terminal running turtle_teleop_key and the turtlesim window. Arrange these windows so that you can see the turtlesim window, but also have the terminal running turtle_teleop_key active so that you can control the turtle in turtlesim.

Use the arrow keys on your keyboard to control the turtle. It will move around the screen, using its attached “pen” to draw the path it followed so far.

!!! note
    Pressing an arrow key will only cause the turtle to move a short distance and then stop. This is because, realistically, you wouldn’t want a robot to continue carrying on an instruction if, for example, the operator lost the connection to the robot

You can see the nodes, and their associated topics, services, and actions, using the list subcommands of the respective commands:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```
You will learn more about these concepts in the coming tutorials. Since the goal of this tutorial is only to get a general overview of turtlesim, you will use rqt to call some of the turtlesim services and interact with turtlesim_node.

> 4 Install rqt

Open a new terminal to install rqt and its plugins:

=== "Linux"

    The standard archive for installing ROS 2 on macOS contains rqt and its plugins, so you should already have rqt installed.

=== "Windows"
    The standard archive for installing ROS 2 on Windows contains rqt and its plugins, so you should already have rqt installed.

To run rqt

```bash
rqt
```

> 5 Use rqt

When running rqt for the first time, the window will be blank. No worries; just select Plugins > Services > Service Caller from the menu bar at the top.

!!! note
    It may take some time for rqt to locate all the plugins. If you click on Plugins but don’t see Services or any other options, you should close rqt and enter the command rqt --force-discover in your terminal.

Use the refresh button to the left of the Service dropdown list to ensure all the services of your turtlesim node are available.

Click on the Service dropdown list to see turtlesim’s services, and select the /spawn service.

[5.1 Try the spawn service]
Let’s use rqt to call the /spawn service. You can guess from its name that /spawn will create another turtle in the turtlesim window.

Give the new turtle a unique name, like turtle2, by double-clicking between the empty single quotes in the Expression column. You can see that this expression corresponds to the value of name and is of type string.

Next enter some valid coordinates at which to spawn the new turtle, like x = 1.0 and y = 1.0.

!!! note
    If you try to spawn a new turtle with the same name as an existing turtle, like the default turtle1, you will get an error message in the terminal running turtlesim_node:

    ```bash
    [ERROR] [turtlesim]: A turtle named [turtle1] already exists
    ```
To spawn turtle2, you then need to call the service by clicking the Call button on the upper right side of the rqt window.

If the service call was successful, you should see a new turtle (again with a random design) spawn at the coordinates you input for x and y.

If you refresh the service list in rqt, you will also see that now there are services related to the new turtle, /turtle2/..., in addition to /turtle1/....

[5.2 Try the set_pen service]

Now let’s give turtle1 a unique pen using the /set_pen service:

The values for r, g and b, which are between 0 and 255, set the color of the pen turtle1 draws with, and width sets the thickness of the line.

To have turtle1 draw with a distinct red line, change the value of r to 255, and the value of width to 5. Don’t forget to call the service after updating the values.

If you return to the terminal where turtle_teleop_key is running and press the arrow keys, you will see turtle1’s pen has changed.

You’ve probably also noticed that there’s no way to move turtle2. That’s because there is no teleop node for turtle2.

[6 Remapping]
ou need a second teleop node in order to control turtle2. However, if you try to run the same command as before, you will notice that this one also controls turtle1. The way to change this behavior is by remapping the cmd_vel topic.

In a new terminal, source ROS 2, and run:

```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

Now, you can move turtle2 when this terminal is active, and turtle1 when the other terminal running turtle_teleop_key is active.

[7 Close turtlesim]
To stop the simulation, you can enter Ctrl + C in the turtlesim_node terminal, and q in the turtle_teleop_key terminals.

##### Summary
Using turtlesim and rqt is a great way to learn the core concepts of ROS 2.

#### Understanding nodes
Goal: Learn about the function of nodes in ROS 2, and the tools to interact with them.

##### Background
[1 The ROS 2 graph]
Over the next few tutorials, you will learn about a series of core ROS 2 concepts that make up what is referred to as the “ROS (2) graph”.

The ROS graph is a network of ROS 2 elements processing data together at the same time. It encompasses all executables and the connections between them if you were to map them all out and visualize them.

[2 Nodes in ROS 2]
Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.

A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

![Nodes-TopicandService](Nodes-TopicandService.gif)

##### Prerequisites
The previous tutorial shows you how to install the turtlesim package used here.

As always, don’t forget to source ROS 2 in every new terminal you open.

##### Tasks

> 1 ros2 run

he command ros2 run launches an executable from a package.

```bash
ros2 run <package_name> <executable_name>
```
To run turtlesim, open a new terminal, and enter the following command:

```bash
ros2 run turtlesim turtlesim_node
```
The turtlesim window will open, as you saw in the previous tutorial.

Here, the package name is turtlesim and the executable name is turtlesim_node.

We still don’t know the node name, however. You can find node names by using ros2 node list

> 2 ros2 node list

ros2 node list will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.

Open a new terminal while turtlesim is still running in the other one, and enter the following command:

```bash
ros2 node list
```
The terminal will return the node name:

```bash
/turtlesim
```
Open another new terminal and start the teleop node with the command:

```bash
ros2 run turtlesim turtle_teleop_key
```
Here, we are referring to the turtlesim package again, but this time we target the executable named turtle_teleop_key.

Return to the terminal where you ran ros2 node list and run it again. You will now see the names of two active nodes:

```bash
/turtlesim
/turtle_teleop_key
```

[2.1 Remapping]
Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. In the last tutorial, you used remapping on turtle_teleop_key to change the cmd_vel topic and target turtle2.

Now, let’s reassign the name of our /turtlesim node. In a new terminal, run the following command:

```bash
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```
Since you’re calling ros2 run on turtlesim again, another turtlesim window will open. However, now if you return to the terminal where you ran ros2 node list, and run it again, you will see three node names:

```bash
/my_turtle
/turtlesim
/teleop_turtle
```

> 3 ros2 node info

Now that you know the names of your nodes, you can access more information about them with:

```bash
ros2 node info <node_name>
```

To examine your latest node, my_turtle, run the following command:

```bash
ros2 node info /my_turtle
```

ros2 node info returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node. The output should look like this:

```bash
/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```
Now try running the same command on the /teleop_turtle node, and see how its connections differ from my_turtle.

You will learn more about the ROS graph connection concepts including the message types in the upcoming tutorials.


##### Summary
A node is a fundamental ROS 2 element that serves a single, modular purpose in a robotics system.

In this tutorial, you utilized nodes created in the turtlesim package by running the executables turtlesim_node and turtle_teleop_key.

You learned how to use ros2 node list to discover active node names and ros2 node info to introspect a single node. These tools are vital to understanding the flow of data in a complex, real-world robot system.

#### Understanding topics
Goal: Use rqt_graph and command line tools to introspect ROS 2 topics.

##### Background

ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages.

![Topic-SinglePublisherandSingleSubscriber.gif](Topic-SinglePublisherandSingleSubscriber.gif)

A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics.

![Topic-MultiplePublisherandMultipleSubscriber.gif](Topic-MultiplePublisherandMultipleSubscriber.gif)

Topics are one of the main ways in which data is moved between nodes and therefore between different parts of the system.

##### Pre-requisites

The previous tutorial provides some useful background information on nodes that is built upon here.

As always, don’t forget to source ROS 2 in every new terminal you open.

##### Tasks

> 1 Setup

By now you should be comfortable starting up turtlesim.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another new terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```
Recall from the previous tutorial that the names of these nodes are /turtlesim and /teleop_turtle by default.

> rqt_graph

Throughout this tutorial, we will use rqt_graph to visualize the changing nodes and topics, as well as the connections between them.

The turtlesim tutorial tells you how to install rqt and all its plugins, including rqt_graph.

To run rqt_graph, open a new terminal and enter the command:

```bash
rqt_graph
```

You can also open rqt_graph by opening rqt and selecting Plugins > Introspection > Node Graph.

You should see the above nodes and topic, as well as two actions around the periphery of the graph (let’s ignore those for now). If you hover your mouse over the topic in the center, you’ll see the color highlighting like in the image above.

The graph is depicting how the /turtlesim node and the /teleop_turtle node are communicating with each other over a topic. The /teleop_turtle node is publishing data (the keystrokes you enter to move the turtle around) to the /turtle1/cmd_vel topic, and the /turtlesim node is subscribed to that topic to receive the data.

The highlighting feature of rqt_graph is very helpful when examining more complex systems with many nodes and topics connected in many different ways.

rqt_graph is a graphical introspection tool. Now we’ll look at some command line tools for introspecting topics.

> 3 ros2 topic list

Running the ros2 topic list command in a new terminal will return a list of all the topics currently active in the system:

```bash
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
ros2 topic list -t will return the same list of topics, this time with the topic type appended in brackets:

```bash
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```
These attributes, particularly the type, are how nodes know they’re talking about the same information as it moves over topics.

If you’re wondering where all these topics are in rqt_graph, you can uncheck all the boxes under Hide:

For now, though, leave those options checked to avoid confusion.

> ros2 topic echo

To see the data being published on a topic, use:

```bash
ros2 topic echo <topic_name>
```

Since we know that /teleop_turtle publishes data to /turtlesim over the /turtle1/cmd_vel topic, let’s use echo to introspect that topic:

```bash
ros2 topic echo /turtle1/cmd_vel
```
At first, this command won’t return any data. That’s because it’s waiting for /teleop_turtle to publish something.

Return to the terminal where turtle_teleop_key is running and use the arrows to move the turtle around. Watch the terminal where your echo is running at the same time, and you’ll see position data being published for every movement you make:

```bash
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

/_ros2cli_26646 is the node created by the echo command we just ran (the number might be different). Now you can see that the publisher is publishing data over the cmd_vel topic, and two subscribers are subscribed to it.

> ros2 topic info

Topics don’t have to only be one-to-one communication; they can be one-to-many, many-to-one, or many-to-many.

Another way to look at this is running:

```bash
ros2 topic info /turtle1/cmd_vel
```

Which will return:

```bash
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```

> 6 ros2 interface show

Nodes send data over topics using messages. Publishers and subscribers must send and receive the same type of message to communicate.

The topic types we saw earlier after running ros2 topic list -t let us know what message type is used on each topic. Recall that the cmd_vel topic has the type:

```bash
geometry_msgs/msg/Twist
```

This means that in the package geometry_msgs there is a msg called Twist.

Now we can run ros2 interface show <msg type> on this type to learn its details. Specifically, what structure of data the message expects.

```bash
ros2 interface show geometry_msgs/msg/Twist
```
For the message type from above it yields:

```bash
# This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```

This tells you that the /turtlesim node is expecting a message with two vectors, linear and angular, of three elements each. If you recall the data we saw /teleop_turtle passing to /turtlesim with the echo command, it’s in the same structure:

```bash
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

> 7 ros2 topic pub

Now that you have the message structure, you can publish data to a topic directly from the command line using:

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```
The '<args>' argument is the actual data you’ll pass to the topic, in the structure you just discovered in the previous section.

It’s important to note that this argument needs to be input in YAML syntax. Input the full command like so:

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
--once is an optional argument meaning “publish one message then exit”.

You will see the following output in the terminal:

```bash
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.8))
```
And you will see the turtle moves a quarter turn to the left.

The turtle (and commonly the real robots which it is meant to emulate) require a steady stream of commands to operate continuously. So, to get the turtle to keep moving, you can run:

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
The difference here is the removal of the --once option and the addition of the --rate 1 option, which tells ros2 topic pub to publish the command in a steady stream at 1 Hz.

You can refresh rqt_graph to see what’s happening graphically. You will see that the ros2 topic pub ... node (/_ros2cli_30358) is publishing over the /turtle1/cmd_vel topic, which is being received by both the ros2 topic echo ... node (/_ros2cli_26646) and the /turtlesim node now.

Finally, you can run echo on the pose topic and recheck rqt_graph:

```bash
ros2 topic echo /turtle1/pose
```
You can see that the /turtlesim node is also publishing to the pose topic, which the new echo node has subscribed to.

When publishing messages with timestamps, pub has two methods to automatically fill them out with the current time. For messages with a std_msgs/msg/Header, the header field can be set to auto to fill out the stamp field.

```bash
ros2 topic pub /pose geometry_msgs/msg/PoseStamped '{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'
```
If the message does not use a full header, but just has a field with type builtin_interfaces/msg/Time, that can be set to the value now.

```bash
ros2 topic pub /reference sensor_msgs/msg/TimeReference '{header: "auto", time_ref: "now", source: "dumy"}'
```

> 8 ros2 topic hz

For one last introspection on this process, you can view the rate at which data is published using:

```bash
ros2 topic hz /turtle1/pose
```
It will return data on the rate at which the /turtlesim node is publishing data to the pose topic.

```bash
average rate: 59.354
  min: 0.005s max: 0.027s std dev: 0.00284s window: 58
```
Recall that you set the rate of turtle1/cmd_vel to publish at a steady 1 Hz using ros2 topic pub --rate 1. If you run the above command with turtle1/cmd_vel instead of turtle1/pose, you will see an average reflecting that rate.

> 9 Clean up

At this point you’ll have a lot of nodes running. Don’t forget to stop them by entering Ctrl+C in each terminal.


##### Summary
Nodes publish information over topics, which allows any number of other nodes to subscribe to and access that information. In this tutorial you examined the connections between several nodes over topics using rqt_graph and command line tools. You should now have a good idea of how data moves around a ROS 2 system.

#### Understanding services
Goal: Learn about services in ROS 2 using command line tools.

##### Background
Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

![Service-SingleServiceClient.gif](Service-SingleServiceClient.gif)

![Service-MultipleServiceClient.gif](Service-MultipleServiceClient.gif)

##### Prerequisites
Some concepts mentioned in this tutorial, like Nodes and Topics, were covered in previous tutorials in the series.

You will need the turtlesim package.

As always, don’t forget to source ROS 2 in every new terminal you open.
##### Tasks

###### 1 Setup
Start up the two turtlesim nodes, /turtlesim and /teleop_turtle.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```
###### 2 ros2 service list
Running the ros2 service list command in a new terminal will return a list of all the services currently active in the system:

```bash
ros2 service list
```
you will see the following output:

```bash
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
/turtlesim/get_parameter_types
/turtlesim/get_parameters
/turtlesim/list_parameters
/turtlesim/set_parameters
/turtlesim/set_parameters_atomically
```
You will see that both nodes have the same six services with parameters in their names. Nearly every node in ROS 2 has these infrastructure services that parameters are built off of. There will be more about parameters in the next tutorial. In this tutorial, the parameter services will be omitted from the discussion.

For now, let’s focus on the turtlesim-specific services, /clear, /kill, /reset, /spawn, /turtle1/set_pen, /turtle1/teleport_absolute, and /turtle1/teleport_relative. You may recall interacting with some of these services using rqt in the Use turtlesim, ros2, and rqt tutorial.

###### 3 ros2 service type
Services have types that describe how the request and response data of a service is structured. Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.

To find out the type of a service, use the command:

```bash
ros2 service type <service_name>
```

Let’s take a look at turtlesim’s /clear service. In a new terminal, enter the command:

```bash
ros2 service type /clear
```

Which should return:

```bash
std_srvs/srv/Empty
```

The Empty type means the service call sends no data when making a request and receives no data when receiving a response.

[ros2 service list -t]
To see the types of all the active services at the same time, you can append the --show-types option, abbreviated as -t, to the list command:

```bash
ros2 service list -t
```
which will return:

```bash
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
...
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
...
```


###### 4 ros2 service find
If you want to find all the services of a specific type, you can use the command:

```bash
ros2 service find <type_name>
```
For example, you can find all the Empty typed services like this:

```bash
ros2 service find std_srvs/srv/Empty
```
Which will return:

```bash
/clear
/reset
```

###### 5 ros2 interface show
You can call services from the command line, but first you need to know the structure of the input arguments.

```bash
ros2 interface show <type_name>
```

Try this on the /clear service’s type, Empty:
```bash
ros2 interface show std_srvs/srv/Empty
```

Which will return:

```bash
---
```

The --- separates the request structure (above) from the response structure (below). But, as you learned earlier, the Empty type doesn’t send or receive any data. So, naturally, its structure is blank.

Let’s introspect a service with a type that sends and receives data, like /spawn. From the results of ros2 service list -t, we know /spawn’s type is turtlesim/srv/Spawn.

To see the request and response arguments of the /spawn service, run the command:

```bash
ros2 interface show turtlesim/srv/Spawn
```

Which will return:

```bash
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

The information above the --- line tells us the arguments needed to call /spawn. x, y and theta determine the 2D pose of the spawned turtle, and name is clearly optional.

The information below the line isn’t something you need to know in this case, but it can help you understand the data type of the response you get from the call.

###### 6 ros2 service call

Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:

```bash
ros2 service call <service_name> <service_type> <arguments>
```

The <arguments> part is optional. For example, you know that Empty typed services don’t have any arguments:

```bash
ros2 service call /clear std_srvs/srv/Empty
```
This command will clear the turtlesim window of any lines your turtle has drawn.

Now let’s spawn a new turtle by calling /spawn and setting arguments. Input <arguments> in a service call from the command-line need to be in YAML syntax.

Enter the command:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```
You will get this method-style view of what’s happening, and then the service response:
```bash
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```
Your turtlesim window will update with the newly spawned turtle right away:

##### Summary
Nodes can communicate using services in ROS 2. Unlike a topic - a one way communication pattern where a node publishes information that can be consumed by one or more subscribers - a service is a request/response pattern where a client makes a request to a node providing the service and the service processes the request and generates a response.

You generally don’t want to use a service for continuous calls; topics or even actions would be better suited.

In this tutorial you used command line tools to identify, introspect, and call services.

#### Understanding parameters
Goal: Learn how to get, set, save and reload parameters in ROS 2.

##### Background
A parameter is a configuration value of a node. You can think of parameters as node settings. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters. For more background on parameters, please see the concept document.
##### Prerequisites
This tutorial uses the turtlesim package.
As always, don’t forget to source ROS 2 in every new terminal you open.
##### Tasks
###### 1 Setup
Start up the two turtlesim nodes, /turtlesim and /teleop_turtle.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```
###### 2 ros2 param list
To see the parameters belonging to your nodes, open a new terminal and enter the command:
```bash
ros2 param list
```

You will see the node namespaces, /teleop_turtle and /turtlesim, followed by each node’s parameters:

```bash
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```

Every node has the parameter use_sim_time; it’s not unique to turtlesim.

Based on their names, it looks like /turtlesim’s parameters determine the background color of the turtlesim window using RGB color values.

To determine a parameter’s type, you can use ros2 param get.
###### 3 ros2 param get
To display the type and current value of a parameter, use the command:
```bash
ros2 param get <node_name> <parameter_name>
```
Let’s find out the current value of /turtlesim’s parameter background_g:
```bash
ros2 param get /turtlesim background_g
```
Which will return the value:
```bash
Integer value is: 86
```
Now you know background_g holds an integer value.

If you run the same command on background_r and background_b, you will get the values 69 and 255, respectively.

###### 4 ros2 param set
To change a parameter’s value at runtime, use the command:
```bash
ros2 param set <node_name> <parameter_name> <value>
```
Let’s change /turtlesim’s background color:
```bash
ros2 param set /turtlesim background_r 150
```
Your terminal should return the message:
```bash
Set parameter successful
```
And the background of your turtlesim window should change colors

Setting parameters with the set command will only change them in your current session, not permanently. However, you can save your settings and reload them the next time you start a node.
###### 5 ros2 param dump
You can view all of a node’s current parameter values by using the command:
```bash
ros2 param dump <node_name>
```

The command prints to the standard output (stdout) by default but you can also redirect the parameter values into a file to save them for later. To save your current configuration of /turtlesim’s parameters into the file turtlesim.yaml, enter the command:

```bash
ros2 param dump /turtlesim > turtlesim.yaml
```

You will find a new file in the current working directory your shell is running in. If you open this file, you’ll see the following content:

```bash
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
```

Dumping parameters comes in handy if you want to reload the node with the same parameters in the future.

###### 6 ros2 param load
You can load parameters from a file to a currently running node using the command:
```bash
ros2 param load <node_name> <parameter_file>
```

To load the turtlesim.yaml file generated with ros2 param dump into /turtlesim node’s parameters, enter the command:

```bash
ros2 param load /turtlesim turtlesim.yaml
```

Your terminal will return the message:

```bash
Set parameter background_b successful
Set parameter background_g successful
Set parameter background_r successful
Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
Set parameter use_sim_time successful
```

!!! note
    Read-only parameters can only be modified at startup and not afterwards, that is why there are some warnings for the “qos_overrides” parameters.

###### 7 Load parameters on node startup
To start the same node using your saved parameter values, use:
```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

This is the same command you always use to start turtlesim, with the added flags --ros-args and --params-file, followed by the file you want to load.

Stop your running turtlesim node, and try reloading it with your saved parameters, using:

```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

The turtlesim window should appear as usual, but with the purple background you set earlier.

!!! note
    When a parameter file is used at node startup, all parameters, including the read-only ones, will be updated.

##### Summary
Nodes have parameters to define their default configuration values. You can get and set parameter values from the command line. You can also save the parameter settings to a file to reload them in a future session.

#### Understanding actions
Goal: Introspect actions in ROS 2.

##### Background
Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a **goal**, **feedback**, and a **result**.

**Actions are built on topics and services.** Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.

**Actions use a client-server model**, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

![Action-SingleActionClient.gif](Action-SingleActionClient.gif)

##### Prerequisites
This tutorial builds off concepts, like nodes and topics, covered in previous tutorials.

This tutorial uses the turtlesim package.

As always, don’t forget to source ROS 2 in every new terminal you open.

##### Tasks

###### 1 Setup
Start up the two turtlesim nodes, /turtlesim and /teleop_turtle.

Open a new terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and run:

```bash
ros2 run turtlesim turtle_teleop_key
```
###### 2 Use actions
When you launch the /teleop_turtle node, you will see the following message in your terminal:

```bash
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
```

Let’s focus on the second line, which corresponds to an action. (The first instruction corresponds to the “cmd_vel” topic, discussed previously in the topics tutorial.)

Notice that the letter keys G|B|V|C|D|E|R|T form a “box” around the F key on a US QWERTY keyboard (if you are not using a QWERTY keyboard, see this link to follow along). Each key’s position around F corresponds to that orientation in turtlesim. For example, the E will rotate the turtle’s orientation to the upper left corner.

Pay attention to the terminal where the /turtlesim node is running. Each time you press one of these keys, you are sending a goal to an action server that is part of the /turtlesim node. The goal is to rotate the turtle to face a particular direction. A message relaying the result of the goal should display once the turtle completes its rotation:

```bash
[INFO] [turtlesim]: Rotation goal completed successfully
```

The `F` key will cancel a goal mid-execution.

Try pressing the `C` key, and then pressing the `F` key before the turtle can complete its rotation. In the terminal where the /turtlesim node is running, you will see the message:

```bash
[INFO] [turtlesim]: Rotation goal canceled
```

Not only can the client-side (your input in the teleop) stop a goal, but the server-side (the `/turtlesim` node) can as well. When the server-side chooses to stop processing a goal, it is said to “abort” the goal.

Try hitting the `D` key, then the `G` key before the first rotation can complete. In the terminal where the `/turtlesim` node is running, you will see the message:

```bash
[WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal
```

This action server chose to abort the first goal because it got a new one. It could have chosen something else, like reject the new goal or execute the second goal after the first one finished. Don’t assume every action server will choose to abort the current goal when it gets a new one.

###### 3 ros2 node info
To see the list of actions a node provides, `/turtlesim` in this case, open a new terminal and run the command:

```bash
ros2 node info /turtlesim
```

Which will return a list of `/turtlesim`’s subscribers, publishers, services, action servers and action clients:

```bash
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

Notice that the `/turtle1/rotate_absolute` action for `/turtlesim` is under `Action Servers`. This means `/turtlesim` responds to and provides feedback for the `/turtle1/rotate_absolute` action.

The `/teleop_turtle` node has the name `/turtle1/rotate_absolute` under `Action Clients` meaning that it sends goals for that action name. To see that, run:

```bash
ros2 node info /teleop_turtle
```

Which will return:

```bash
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

###### 4 ros2 action list
To identify all the actions in the ROS graph, run the command:
```bash
ros2 action list
```

Which will return:

```bash
/turtle1/rotate_absolute
```

This is the only action in the ROS graph right now. It controls the turtle’s rotation, as you saw earlier. You also already know that there is one action client (part of /teleop_turtle) and one action server (part of /turtlesim) for this action from using the ros2 node info <node_name> command.

[ros2 action list -t]

Actions have types, similar to topics and services. To find /turtle1/rotate_absolute’s type, run the command:

```bash
ros2 action list -t
```

Which will return:

```bash
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

In brackets to the right of each action name (in this case only /turtle1/rotate_absolute) is the action type, turtlesim/action/RotateAbsolute. You will need this when you want to execute an action from the command line or from code.

###### 5 ros2 action info
You can further introspect the `/turtle1/rotate_absolute` action with the command:

```bash
ros2 action info /turtle1/rotate_absolute
```

Which will return

```bash
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

This tells us what we learned earlier from running `ros2 node info` on each node: The `/teleop_turtle` node has an action client and the `/turtlesim` node has an action server for the `/turtle1/rotate_absolute` action.

###### 6 ros2 interface show
One more piece of information you will need before sending or executing an action goal yourself is the structure of the action type.

Recall that you identified `/turtle1/rotate_absolute`’s type when running the command `ros2 action list -t`. Enter the following command with the action type in your terminal:

```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

Which will return:

```bash
The desired heading in radians
float32 theta
---
The angular displacement in radians to the starting position
float32 delta
---
The remaining rotation in radians
float32 remaining
```

The section of this message above the first `---` is the structure (data type and name) of the **goal request**. The next section is the structure of the **result**. The last section is the structure of the **feedback**.

###### 7 ros2 action send_goal
Now let’s send an action goal from the command line with the following syntax:

```bash
ros2 action send_goal <action_name> <action_type> <values>
```

<values> need to be in YAML format.

Keep an eye on the turtlesim window, and enter the following command into your terminal:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

You should see the turtle rotating, as well as the following message in your terminal:
```bash
Waiting for an action server to become available...
Sending goal:
   theta: 1.57

Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

Result:
  delta: -1.568000316619873

Goal finished with status: SUCCEEDED
```

All goals have a unique ID, shown in the return message. You can also see the result, a field with the name delta, which is the displacement to the starting position.

To see the feedback of this goal, add --feedback to the ros2 action send_goal command:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

Your terminal will return the message:

```bash
Sending goal:
   theta: -1.57

Goal accepted with ID: e6092c831f994afda92f0086f220da27

Feedback:
  remaining: -3.1268222332000732

Feedback:
  remaining: -3.1108222007751465

…

Result:
  delta: 3.1200008392333984

Goal finished with status: SUCCEEDED
```

You will continue to receive feedback, the remaining radians, until the goal is complete.

##### Summary
Actions are like services that allow you to execute **long running tasks**, provide regular feedback, and are cancelable.

A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.

Turtlesim has an action server that action clients can send goals to for rotating turtles. In this tutorial, you introspected that action, `/turtle1/rotate_absolute`, to get a better idea of what actions are and how they work.

#### Using rqt_console to view logs
Goal: Get to know rqt_console, a tool for introspecting log messages.

##### Background

##### Prerequisites

##### Tasks

###### 1 Setup

###### 2 Messages on rqt_console

###### 3 Logger levels
[Set the default logger level]

##### Summary
`rqt_console` can be very helpful if you need to closely examine the log messages from your system. You might want to examine log messages for any number of reasons, usually to find out where something went wrong and the series of events leading up to that.








#### Launching nodes










#### Recording and playing back data









### Beginner: Client Libraries

### Intermediate

### Advanced

### Demos

### Miscellaneous

## 4. How-to Guides

## 5. Concepts

## 6. Contact

## 7. The ROS2 Project

## 8. API Documentation

## 9. Related Projects

## 10. Glossary

## 11. Citations
# ROS2-Humble Guide - Installation

Last Updated: Jan 03, 2024

[🌐 Link to the original page](https://docs.ros.org/en/humble/index.html)

## Ubuntu

### Resources
~

### Set locale
Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
### Setup Sources
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

### Install ROS 2 packages

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
### Environment setup

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
### Try some examples

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

### Next steps after installing
~
### Using the ROS 1 bridge
~
### Additional RMW implementations (optional)
~
### Troubleshooting
~
### Uninstall
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
## Windows
skipped
## RHEL
skipped
## Alternatives
skipped
A list of alternative ways to install ROS 2 – whether it’s by building from source or installing a binary.

## Maintaining Source Checkout
If you installled ROS 2 from source, you can update your source checkout according to this section. Please check the original link.

skipped
## Testing with Pre-release Binaries
skipped

## DDS Implementations

By default, ROS 2 uses DDS as its middleware. It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors. There is currently support for eProsima’s Fast DDS, RTI’s Connext DDS, Eclipse Cyclone DDS, and GurumNetworks GurumDDS. See https://ros.org/reps/rep-2000.html for supported DDS vendors by distribution.


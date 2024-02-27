# ROS2-Humble Guide - Tutorial - Beginner: Client Libraries

[🌐 Original link](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

## I Using colcon To Build Packages
### Background

!!! info
    `colcon` is an iteration on the ROS build tools `catkin_make`, `catkin_make_isolated`, `catkin_tools` and `ament_tools`. For more information on the design of colcon see this [document](https://design.ros2.org/articles/build_tool.html). The source code can be found in the [colcon GitHub organization](https://github.com/colcon).

### Prerequisites
#### 1 Install colcon

=== "LINUX"
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

=== "MACOS"
    ```bash
    python3 -m pip install colcon-common-extensions
    ```

== "WINDOWS"
    ```bash
    pip install -U colcon-common-extensions
    ```

#### 2 Install ROS 2

Follow the [instructions](https://docs.ros.org/en/humble/Installation.html).

### Basics

!!! info
    A ROS workspace is **a directory with a particular structure**. Commonly there is a `src` subdirectory. Inside that subdirectory is where the source code of ROS packages will be located. Typically the directory starts otherwise empty.

colcon does out of source builds. By default it will create the following directories as peers of the `src` directory:

The `build` directory will be where intermediate files are stored. For each package a subfolder will be created in which e.g. CMake is being invoked.

The `install` directory is where each package will be installed to. By default each package will be installed into a separate subdirectory.

The `log` directory contains various logging information about each colcon invocation.

!!! note
    Compared to catkin there is no `devel` directory.

#### 1 Create a workspace

=== "LINUX"
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

=== "MACOS"
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

== "WINDOWS"
    ```bash
    md \dev\ros2_ws\src
    cd \dev\ros2_ws
    ```


#### 2 Add some sources
Let’s clone the examples repository into the `src` directory of the workspace:

```bash
git clone https://github.com/ros2/examples src/examples -b humble
```


#### 3 Source an underlay
It is important that we have sourced the environment for an existing ROS 2 installation that will provide our workspace with the necessary build dependencies for the example packages. This is achieved by sourcing the setup script provided by a binary installation or a source installation, ie. another colcon workspace (see Installation). We call this environment an **underlay**.

Our workspace, ros2_ws, will be an **overlay** on top of the existing ROS 2 installation. In general, it is recommended to use an overlay when you plan to iterate on a small number of packages, rather than putting all of your packages into the same workspace.


#### 4 Build the workspace
!!! attention
    To build packages on Windows you need to be in a Visual Studio environment, see **Building the ROS 2 Code** for more details.

In the root of the workspace, run `colcon build`. Since build types such as `ament_cmake` do not support the concept of the `devel` space and require the package to be installed, colcon supports the option `--symlink-install`. This allows the installed files to be changed by changing the files in the `source` space (e.g. Python files or other non-compiled resources) for faster iteration.

=== "LINUX"
    ```bash
    colcon build --symlink-install
    ```

=== "MACOS"
    ```bash
    colcon build --symlink-install
    ```

== "WINDOWS"
    ```bash
    colcon build --symlink-install --merge-install
    ```


#### 5 Run tests
To run tests for the packages we just built, run the following:

Remember to use a x64 Native Tools Command Prompt for VS 2019 for executing the following command, as we are going to build a workspace. You also need to specify --merge-install here since we used it for building above.

=== "LINUX"
    ```bash
    colcon test
    ```

=== "MACOS"
    ```bash
    colcon test
    ```

== "WINDOWS"
    ```bash
    colcon test --merge-install
    ```

colcon test --merge-install

```bash
colcon test --merge-install
```

#### 6 Source the environment
When colcon has completed building successfully, the output will be in the `install` directory. Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. colcon will have generated bash/bat files in the `install` directory to help set up the environment. These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.

=== "LINUX"
    ```bash
    source install/setup.bash
    ```

=== "MACOS"
    ```bash
    . install/setup.bash
    ```

== "WINDOWS"
    ```bash
    call install\setup.bat
    ```
    or with Powershell
    ```bash
    install\setup.ps1
    ```

#### 7 Try a demo
With the environment sourced, we can run executables built by colcon. Let’s run a subscriber node from the examples:

```bash
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
```
In another terminal, let’s run a publisher node (don’t forget to source the setup script):

```bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function
```
You should see messages from the publisher and subscriber with numbers incrementing.

### Create your own package
colcon uses the `package.xml` specification defined in REP 149 (format 2 is also supported).

colcon supports multiple build types. The recommended build types are `ament_cmake` and `ament_python`. Also supported are pure `cmake` packages.

An example of an `ament_python` build is the [ament_index_python package](https://github.com/ament/ament_index/tree/humble/ament_index_python) , where the setup.py is the primary entry point for building.

A package such as [demo_nodes_cpp](https://github.com/ros2/demos/tree/humble/demo_nodes_cpp) uses the `ament_cmake` build type, and uses CMake as the build tool.

For convenience, you can use the tool `ros2 pkg create` to create a new package based on a template.

!!! note
    For `catkin` users, this is the equivalent of `catkin_create_package`.


### Setup colcon_cd
The command `colcon_cd` allows you to quickly change the current working directory of your shell to the directory of a package. As an example `colcon_cd some_ros_package` would quickly bring you to the directory `~/ros2_ws/src/some_ros_package`.


=== "LINUX"
    ```bash
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
    ```

=== "MACOS"
    ```bash
    echo "source /usr/local/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc
    ```

== "WINDOWS"
    not yet available


Depending on the way you installed `colcon_cd` and where your workspace is, the instructions above may vary, please refer to the documentation for more details. To undo this in Linux and macOS, locate your system’s shell startup script and remove the appended source and export commands.

### Setup colcon tab completion
The command `colcon` [supports command completion](https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion) for bash and bash-like shells if the `colcon-argcomplete` package is installed.

=== "LINUX"
    ```bash
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    ```

=== "MACOS"
    ```bash
    echo "source $HOME/.local/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bash_profile
    ```

== "WINDOWS"
    not yet available

Depending on the way you installed `colcon` and where your workspace is, the instructions above may vary, please refer to the [documentation](https://colcon.readthedocs.io/en/released/user/installation.html) for more details. To undo this in Linux and macOS, locate your system’s shell startup script and remove the appended source command.

### Tips
If you do not want to build a specific package place an empty file named `COLCON_IGNORE` in the directory and it will not be indexed.

If you want to avoid configuring and building tests in CMake packages you can pass: `--cmake-args -DBUILD_TESTING=0`.

If you want to run a single particular test from a package:

```bash
colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
```

## II Creating A Workspace
### Background
A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

You also have the option of sourcing an “overlay” - a secondary workspace where you can add new packages without interfering with the existing ROS 2 workspace that you’re extending, or “underlay”. Your underlay must contain the dependencies of all the packages in your overlay. Packages in your overlay will override packages in the underlay. It’s also possible to have several layers of underlays and overlays, with each successive overlay using the packages of its parent underlays.

### Prerequisites
- ROS 2 installation
- colcon installation
- git installation
- turtlesim installation
- Have rosdep installed
- Understanding of basic terminal commands (here’s a guide for Linux)
- Text editor of your choice

### Tasks

#### 1 Source ROS 2 environment
Your main ROS 2 installation will be your underlay for this tutorial. (Keep in mind that an underlay does not necessarily have to be the main ROS 2 installation.)

Depending on how you installed ROS 2 (from source or binaries), and which platform you’re on, your exact source command will vary:

=== "LINUX"
    ```bash
    source /opt/ros/humble/setup.bash
    ```

=== "MACOS"
    ```bash
    . ~/ros2_install/ros2-osx/setup.bash
    ```

== "WINDOWS"
    Remember to use a x64 Native Tools Command Prompt for VS 2019 for executing the following commands, as we are going to build a workspace.
    ```bash
    call C:\dev\ros2_install\local_setup.bat
    ```

#### 2 Create a new directory
Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name `ros2_ws`, for “development workspace”:

=== "LINUX"
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

=== "MACOS"
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

== "WINDOWS"
    ```bash
    md \dev\ros2_ws\src
    cd \dev\ros2_ws
    ```

Another best practice is to put any packages in your workspace into the src directory. The above code creates a src directory inside ros2_ws and then navigates into it.

#### 3 Clone a sample repo
Ensure you’re still in the `ros2_ws/src` directory before you clone.

In the rest of the beginner developer tutorials, you will create your own packages, but for now you will practice putting a workspace together using existing packages.

If you went through the Beginner: CLI Tools tutorials, you’ll be familiar with `turtlesim`, one of the packages in ros_tutorials.

A repo can have multiple branches. You need to check out the one that targets your installed ROS 2 distro. When you clone this repo, add the -b argument followed by that branch.

In the ros2_ws/src directory, run the following command:

```bash
git clone https://github.com/ros/ros_tutorials.git -b humble
```

Now `ros_tutorials` is cloned in your workspace. The `ros_tutorials` repository contains the `turtlesim` package, which we’ll use in the rest of this tutorial. The other packages in this repository are not built because they contain a `COLCON_IGNORE` file.

So far you have populated your workspace with a sample package, but it isn’t a fully-functional workspace yet. You need to resolve the dependencies first and then build the workspace.

#### 4 Resolve dependencies
Before building the workspace, you need to resolve the package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.

From the root of your workspace (`ros2_ws`), run the following command:

=== "LINUX"
    ```bash
    rosdep install --from-paths src --ignore-src --rosdistro humble -y
    ```

=== "MACOS"
    rosdep only runs on Linux, so you can skip ahead to section “5 Build the workspace with colcon”.

=== "WINDOWS"
    rosdep only runs on Linux, so you can skip ahead to section “5 Build the workspace with colcon”.

If you installed ROS 2 on Linux from source or the “fat” archive, you will need to use the rosdep command from their installation instructions. Here are the from-source rosdep section and the “fat” archive rosdep section.

If you already have all your dependencies, the console will return:

```bash
All required rosdeps installed successfully
```

Packages declare their dependencies in the package.xml file (you will learn more about packages in the next tutorial). This command walks through those declarations and installs the ones that are missing. You can learn more about rosdep in another tutorial (coming soon).

#### 5 Build the workspace with colcon
From the root of your workspace (ros2_ws), you can now build your packages using the command:

=== "LINUX"
    ```bash
    colcon build
    ```

=== "MACOS"
    ```bash
    colcon build
    ```

== "WINDOWS"
    Windows doesn’t allow long paths, so merge-install will combine all the paths into the install directory.
    ```bash
    colcon build --merge-install
    ```

The console will return the following message:

```bash
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]

Summary: 1 package finished [5.58s]
```

!!! note
    Other useful arguments for colcon build:
    --packages-up-to builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
    --symlink-install saves you from having to rebuild every time you tweak python scripts
    --event-handlers console_direct+ shows console output while building (can otherwise be found in the log directory)

Once the build is finished, enter the command in the workspace root (~/ros2_ws):

=== "LINUX"
    ```bash
    ls
    ```

=== "MACOS"
    ```bash
    ls
    ```

== "WINDOWS"
    ```bash
    dir
    ```

And you will see that colcon has created new directories: build, install, log and src. The install directory is where your workspace’s setup files are, which you can use to source your overlay.

#### 6 Source the overlay
Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it:

=== "LINUX"
    ```bash
    source /opt/ros/humble/setup.bash
    ```

=== "MACOS"
    ```bash
    . ~/ros2_install/ros2-osx/setup.bash
    ```

== "WINDOWS"
    In this case you can use a normal command prompt, as we are not going to build any workspace in this terminal.
    ```bash
    call C:\dev\ros2_install\local_setup.bat
    ```

Go into the root of your workspace (ros2_ws):

=== "LINUX"
    ```bash
    cd ~/ros2_ws
    ```

=== "MACOS"
    ```bash
    cd ~/ros2_ws
    ```

== "WINDOWS"
    ```bash
    cd \dev\ros2_ws
    ```
In the root, source your overlay:

=== "LINUX"
    ```bash
    source install/local_setup.bash
    ```

=== "MACOS"
    ```bash
    . install/local_setup.bash
    ```

== "WINDOWS"
    ```bash
    call install\setup.bat
    ```

!!! note
    Sourcing the local_setup of the overlay will only add the packages available in the overlay to your environment. setup sources the overlay as well as the underlay it was created in, allowing you to utilize both workspaces.

    So, sourcing your main ROS 2 installation’s setup and then the ros2_ws overlay’s local_setup, like you just did, is the same as just sourcing ros2_ws’s setup, because that includes the environment of its underlay.

Now you can run the turtlesim package from the overlay:

```bash
ros2 run turtlesim turtlesim_node
```
But how can you tell that this is the overlay turtlesim running, and not your main installation’s turtlesim?

Let’s modify turtlesim in the overlay so you can see the effects:

- You can modify and rebuild packages in the overlay separately from the underlay.
- The overlay takes precedence over the underlay.

#### 7 Modify the overlay
You can modify `turtlesim` in your overlay by editing the title bar on the turtlesim window. To do this, locate the `turtle_frame.cpp` file in `~/ros2_ws/src/ros_tutorials/turtlesim/src`. Open `turtle_frame.cpp` with your preferred text editor.

On line 52 you will see the function `setWindowTitle("TurtleSim")`;. Change the value `"TurtleSim"` to `"MyTurtleSim"`, and save the file.

Return to the first terminal where you ran `colcon build` earlier and run it again.

Return to the second terminal (where the overlay is sourced) and run turtlesim again:

```bash
ros2 run turtlesim turtlesim_node
```

You will see the title bar on the turtlesim window now says “MyTurtleSim”.

Even though your main ROS 2 environment was sourced in this terminal earlier, the overlay of your `ros2_ws` environment takes precedence over the contents of the underlay.

To see that your underlay is still intact, open a brand new terminal and source only your ROS 2 installation. Run turtlesim again:
```bash
ros2 run turtlesim turtlesim_node
```
You can see that modifications in the overlay did not actually affect anything in the underlay.
### Summary
In this tutorial, you sourced your main ROS 2 distro install as your underlay, and created an overlay by cloning and building packages in a new workspace. The overlay gets prepended to the path, and takes precedence over the underlay, as you saw with your modified turtlesim.

Using overlays is recommended for working on a small number of packages, so you don’t have to put everything in the same workspace and rebuild a huge workspace on every iteration.



## III Creating A Package

## IV Writing A Simple Publisher And Subscriber (C++)

## V Writing A Simple Publisher And Subscriber (Python)

## VI Writing A Simple Service And Client (C++)

## VII Writing A Simple Service And Client (Python)

## VIII Creating Custom Msg and Srv Files

## IX Implementing Custom Interfaces

## X Using Parameters In A Class (C++)

## XI Using Parameters In A Class (Python)

## XII Using ros2doctor To Identify Issues

## XIII Creating And Using Plugins
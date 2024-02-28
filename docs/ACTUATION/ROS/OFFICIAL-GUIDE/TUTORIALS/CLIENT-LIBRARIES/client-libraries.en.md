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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
=== "WINDOWS"
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
### Background
#### 1 What is a ROS 2 package?
A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.

Package creation in ROS 2 uses `ament` as its build system and `colcon` as its build tool. You can create a package using either `CMake` or `Python`, which are officially supported, though other build types do exist.

#### 2 What makes up a ROS 2 package?
ROS 2 Python and CMake packages each have their own minimum required contents:

=== "CMake"
    - CMakeLists.txt file that describes how to build the code within the package
    - include/<package_name> directory containing the public headers for the package
    - package.xml file containing meta information about the package
    - src directory containing the source code for the package
=== "Python"
    - package.xml file containing meta information about the package
    - resource/<package_name> marker file for the package
    - setup.cfg is required when a package has executables, so ros2 run can find them
    - setup.py containing instructions for how to install the package
    - <package_name> - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py

The simplest possible package may have a file structure that looks like:

=== "CMake"
    ```bash
    my_package/
         CMakeLists.txt
         include/my_package/
         package.xml
         src/
    ```
=== "Python"
    ```bash
    my_package/
          package.xml
          resource/my_package
          setup.cfg
          setup.py
          my_package/    
    ```

#### 3 Packages in a workspace
A single workspace can contain as many packages as you want, each in their own folder. You can also have packages of different build types in one workspace (CMake, Python, etc.). You cannot have nested packages.

Best practice is to have a `src` folder within your workspace, and to create your packages in there. This keeps the top level of the workspace “clean”.

A trivial workspace might look like:

```bash
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```


### Prerequisites
You should have a ROS 2 workspace after following the instructions in the previous tutorial. You will create your package in this workspace.

### Tasks
#### 1 Create a package
First, source your ROS 2 installation.

Let’s use the workspace you created in the previous tutorial, ros2_ws, for your new package.

Make sure you are in the src folder before running the package creation command.

=== "LINUX"
    ```bash
    cd ~/ros2_ws/src
    ```
=== "MACOS"
    ```bash
    cd ~/ros2_ws/src
    ```
=== "WINDOWS"
    ```bash
    cd \ros2_ws\src
    ```

The command syntax for creating a new package in ROS 2 is:

=== "CMake"
    ```bash
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
    ```
=== "Python"
    ```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
    ```
For this tutorial, you will use the optional argument --node-name which creates a simple Hello World type executable in the package.

Enter the following command in your terminal:

=== "CMake"
    ```bash
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
    ```

=== "Python"
    ```bash
    ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package
    ```
You will now have a new folder within your workspace’s src directory called my_package.

After running the command, your terminal will return the message:

=== "CMake"
    ```bash
    going to create a new package
    package name: my_package
    destination directory: /home/user/ros2_ws/src
    package format: 3
    version: 0.0.0
    description: TODO: Package description
    maintainer: ['<name> <email>']
    licenses: ['TODO: License declaration']
    build type: ament_cmake
    dependencies: []
    node_name: my_node
    creating folder ./my_package
    creating ./my_package/package.xml
    creating source and include folder
    creating folder ./my_package/src
    creating folder ./my_package/include/my_package
    creating ./my_package/CMakeLists.txt
    creating ./my_package/src/my_node.cpp
    ```

=== "Python"
    ```bash
    going to create a new package
    package name: my_package
    destination directory: /home/user/ros2_ws/src
    package format: 3
    version: 0.0.0
    description: TODO: Package description
    maintainer: ['<name> <email>']
    licenses: ['TODO: License declaration']
    build type: ament_python
    dependencies: []
    node_name: my_node
    creating folder ./my_package
    creating ./my_package/package.xml
    creating source folder
    creating folder ./my_package/my_package
    creating ./my_package/setup.py
    creating ./my_package/setup.cfg
    creating folder ./my_package/resource
    creating ./my_package/resource/my_package
    creating ./my_package/my_package/__init__.py
    creating folder ./my_package/test
    creating ./my_package/test/test_copyright.py
    creating ./my_package/test/test_flake8.py
    creating ./my_package/test/test_pep257.py
    creating ./my_package/my_package/my_node.py
    ```

You can see the automatically generated files for the new package.





#### 2 Build a package
Putting packages in a workspace is especially valuable because you can build many packages at once by running `colcon build` in the workspace root. Otherwise, you would have to build each package individually.

Return to the root of your workspace:

=== "LINUX"
    ```bash
    cd ~/ros2_ws
    ```
=== "MACOS"
    ```bash
    cd ~/ros2_ws
    ```
=== "WINDOWS"
    ```bash
    cd \ros2_ws
    ```

Now you can build your packages:

=== "LINUX"
    ```bash
    colcon build
    ```
=== "MACOS"
    ```bash
    colcon build
    ```
=== "WINDOWS"
    ```bash
    colcon build --merge-install
    ```

Recall from the last tutorial that you also have the `ros_tutorials` packages in your `ros2_ws`. You might have noticed that running `colcon build` also built the `turtlesim` package. That’s fine when you only have a few packages in your workspace, but when there are many packages, colcon build can take a long time.

To build only the my_package package next time, you can run:

```bash
colcon build --packages-select my_package
```

#### 3 Source the setup file
To use your new package and executable, first open a new terminal and source your main ROS 2 installation.

Then, from inside the ros2_ws directory, run the following command to source your workspace:

=== "LINUX"
    ```bash
    source install/local_setup.bash
    ```
=== "MACOS"
    ```bash
    . install/local_setup.bash
    ```
=== "WINDOWS"
    ```bash
    call install/local_setup.bat
    ```

Now that your workspace has been added to your path, you will be able to use your new package’s executables.
#### 4 Use the package
To run the executable you created using the `--node-name` argument during package creation, enter the command:

```bash
ros2 run my_package my_node
```

Which will return a message to your terminal:

=== "CMake"
    ```bash
    hello world my_package package
    ```
=== "Python"
    ```bash
    hello world my_package package
    ```

#### 5 Examine package contents
Inside `ros2_ws/src/my_package`, you will see the files and folders that `ros2 pkg create` automatically generated:

=== "CMake"
    ```bash
    CMakeLists.txt  include  package.xml  src
    ```
    `my_node.cpp` is inside the `src` directory. This is where all your custom C++ nodes will go in the future.
=== "Python"
    ```bash
    my_package  package.xml  resource  setup.cfg  setup.py  test
    ```
    `my_node.py` is inside the `my_package` directory. This is where all your custom Python nodes will go in the future.

#### 6 Customize package.xml
You may have noticed in the return message after creating your package that the fields `description` and `license` contain `TODO` notes. That’s because the package description and license declaration are not automatically set, but are required if you ever want to release your package. The `maintainer` field may also need to be filled in.

From `ros2_ws/src/my_package`, open `package.xml` using your preferred text editor:

=== "CMake"
    ```bash
    <?xml version="1.0"?>
    <?xml-model
       href="http://download.ros.org/schema/package_format3.xsd"
       schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
     <name>my_package</name>
     <version>0.0.0</version>
     <description>TODO: Package description</description>
     <maintainer email="user@todo.todo">user</maintainer>
     <license>TODO: License declaration</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
    </package>
    ```
=== "Python"
    ```bash
    <?xml version="1.0"?>
    <?xml-model
       href="http://download.ros.org/schema/package_format3.xsd"
       schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
     <name>my_package</name>
     <version>0.0.0</version>
     <description>TODO: Package description</description>
     <maintainer email="user@todo.todo">user</maintainer>
     <license>TODO: License declaration</license>

     <test_depend>ament_copyright</test_depend>
     <test_depend>ament_flake8</test_depend>
     <test_depend>ament_pep257</test_depend>
     <test_depend>python3-pytest</test_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
    </package>
    ```

Input your name and email on the `maintainer` line if it hasn’t been automatically populated for you. Then, edit the description line to summarize the package:

```bash
<description>Beginner client libraries tutorials practice package</description>
```
Then, update the `license` line. You can read more about open source licenses here. Since this package is only for practice, it’s safe to use any license. We’ll use `Apache License 2.0`:

```bash
<license>Apache License 2.0</license>
```
Don’t forget to save once you’re done editing.

Below the license tag, you will see some tag names ending with `_depend`. This is where your `package.xml` would list its dependencies on other packages, for colcon to search for. `my_package` is simple and doesn’t have any dependencies, but you will see this space being utilized in upcoming tutorials.

=== "CMake"
    you are done

=== "Python"
    The `setup.py` file contains the same description, maintainer and license fields as `package.xml`, so you need to set those as well. They need to match exactly in both files. The version and name (`package_name`) also need to match exactly, and should be automatically populated in both files.

    Open setup.py with your preferred text editor.

    ```python
    from setuptools import setup

    package_name = 'my_py_pkg'

    setup(
     name=package_name,
     version='0.0.0',
     packages=[package_name],
     data_files=[
         ('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
         ('share/' + package_name, ['package.xml']),
       ],
     install_requires=['setuptools'],
     zip_safe=True,
     maintainer='TODO',
     maintainer_email='TODO',
     description='TODO: Package description',
     license='TODO: License declaration',
     tests_require=['pytest'],
     entry_points={
         'console_scripts': [
                 'my_node = my_py_pkg.my_node:main'
         ],
       },
    )    
    ```
    Edit the maintainer, maintainer_email, and description lines to match package.xml.

    Don’t forget to save the file.

### Summary
You’ve created a package to organize your code and make it easy to use for others.

Your package was automatically populated with the necessary files, and then you used colcon to build it so you can use its executables in your local environment.

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
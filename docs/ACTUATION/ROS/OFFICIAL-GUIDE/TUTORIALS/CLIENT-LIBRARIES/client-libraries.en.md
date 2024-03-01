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
### Background
[Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a [topic](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html). The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

The code used in these examples can be found [here](https://github.com/ros2/examples/tree/humble/rclcpp/topics).

### Prerequisites
In previous tutorials, you learned how to create a workspace and create a package.

### Tasks
#### 1 Create a package
Open a new terminal and [source your ROS 2 installation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) so that `ros2` commands will work.

Navigate into the `ros2_ws` directory created in a previous tutorial.

Recall that packages should be created in the `src` directory, not the root of the workspace. So, navigate into `ros2_ws/src`, and run the package creation command:

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub
```

Your terminal will return a message verifying the creation of your package `cpp_pubsub` and all its necessary files and folders.

Navigate into `ros2_ws/src/cpp_pubsub/src`. Recall that this is the directory in any CMake package where the source files containing executables belong.


#### 2 Write the publisher node
Download the example talker code by entering the following command:

=== "LINUX"
    ```bash
    wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp
    ```

=== "MACOS"
    ```bash
    wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp
    ```

=== "WINDOWS"
    In a Windows command line prompt:
    ```bash
    curl -sk https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp -o publisher_member_function.cpp
    ```
    or with Powershell:
    ```bash
    curl https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp -o publisher_member_function.cpp
    ```
Now there will be a new file named publisher_member_function.cpp. Open the file using your preferred text editor.

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
##### 2.1 Examine the code
The top of the code includes the standard C++ headers you will be using. After the standard C++ headers is the `rclcpp/rclcpp.hpp` include which allows you to use the most common pieces of the ROS 2 system. Last is `std_msgs/msg/string.hpp`, which includes the built-in message type you will use to publish data.

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
```
These lines represent the node’s dependencies. Recall that dependencies have to be added to `package.xml` and `CMakeLists.txt`, which you’ll do in the next section.

The next line creates the node class `MinimalPublisher` by **inheriting** from `rclcpp::Node`. Every this in the code is referring to the node.

```cpp
class MinimalPublisher : public rclcpp::Node
```
The public constructor names the node `minimal_publisher` and initializes `count_` to 0. Inside the constructor, the publisher is initialized with the `String` message type, the topic name `topic`, and the required queue size to limit messages in the event of a backup. Next, `timer_` is initialized, which causes the `timer_callback` function to be executed twice a second.

```cpp
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
```
The `timer_callback` function is where the message data is set and the messages are actually published. The `RCLCPP_INFO` macro ensures every published message is printed to the console.

```cpp
private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
```
Last is the declaration of the timer, publisher, and counter fields.

```cpp
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
```
Following the `MinimalPublisher` class is `main`, where the node actually executes. `rclcpp::init` initializes ROS 2, and `rclcpp::spin` starts processing data from the node, including callbacks from the timer.

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

##### 2.2 Add dependencies
Navigate one level back to the `ros2_ws/src/cpp_pubsub` directory, where the `CMakeLists.txt` and `package.xml` files have been created for you.

Open `package.xml` with your text editor.

As mentioned in the previous tutorial, make sure to fill in the <description>, <maintainer> and <license> tags:

```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
Add a new line after the ament_cmake buildtool dependency and paste the following dependencies corresponding to your node’s include statements:
    
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

This declares the package needs `rclcpp` and `std_msgs` when its code is built and executed.

Make sure to save the file.

##### 2.3 CMakeLists.txt
Now open the `CMakeLists.txt` file. Below the existing dependency `find_package(ament_cmake REQUIRED)`, add the lines:

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```
After that, add the executable and name it `talker` so you can run your node using ros2 run:

```cmake
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

Finally, add the `install(TARGETS...)` section so `ros2 run` can find your executable:

```cmake
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

You can clean up your `CMakeLists.txt` by removing some unnecessary sections and comments, so it looks like this:

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

You could build your package now, source the local setup files, and run it, but let’s create the subscriber node first so you can see the full system at work.

#### 3 Write the subscriber node

Return to `ros2_ws/src/cpp_pubsub/src` to create the next node. Enter the following code in your terminal:

=== "LINUX"
    ```bash
    wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp
    ```

=== "MACOS"
    ```bash
    wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp
    ```

=== "WINDOWS"
    In a Windows command line prompt:
    ```bash
    curl -sk https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp -o subscriber_member_function.cpp
    ```
    or with Powershell:
    ```bash
    curl https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp -o subscriber_member_function.cpp
    ```
Check to ensure that these files exist:

```bash
publisher_member_function.cpp  subscriber_member_function.cpp
```

Open the subscriber_member_function.cpp with your text editor.

```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

##### 3.1 Examine the code
The subscriber node’s code is nearly identical to the publisher’s. Now the node is named `minimal_subscriber`, and the constructor uses the node’s `create_subscription` class to execute the callback.

There is no timer because the subscriber simply responds whenever data is published to the `topic` topic.

```cpp
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
```

Recall from the [topic tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) that the topic name and message type used by the publisher and subscriber must **match** to allow them to communicate.

The `topic_callback` function receives the string message data published over the topic, and simply writes it to the console using the `RCLCPP_INFO` macro.

The only field declaration in this class is the subscription.

```cpp
private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```

The `main` function is exactly the same, except now it spins the `MinimalSubscriber` node. For the publisher node, spinning meant starting the timer, but for the subscriber it simply means preparing to receive messages whenever they come.

Since this node has the same dependencies as the publisher node, there’s nothing new to add to `package.xml`.

##### 3.2 CMakeLists.txt
Reopen `CMakeLists.txt` and add the executable and target for the subscriber node below the publisher’s entries.

```cmake
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

Make sure to save the file, and then your pub/sub system should be ready.


#### 4 Build and run
You likely already have the `rclcpp` and `std_msgs` packages installed as part of your ROS 2 system. It’s good practice to run `rosdep` in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

=== "LINUX"
    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    ```

=== "MACOS"
    rosdep only runs on Linux, so you can skip ahead to next step.

=== "WINDOWS"
    rosdep only runs on Linux, so you can skip ahead to next step.

Still in the root of your workspace, `ros2_ws`, build your new package:

=== "LINUX"
    ```bash
    colcon build --packages-select cpp_pubsub
    ```
=== "MACOS"
    ```bash
    colcon build --packages-select cpp_pubsub
    ```
=== "WINDOWS"
    ```bash
    colcon build --merge-install --packages-select cpp_pubsub
    ```
Open a new terminal, navigate to `ros2_ws`, and source the setup files:

=== "LINUX"
    ```bash
    . install/setup.bash
    ```
=== "MACOS"
    ```bash
    . install/setup.bash
    ```
=== "WINDOWS"
    ```bash
    call install\setup.bat
    ```
The terminal should start publishing info messages every 0.5 seconds, like so:

```bash
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

Open another terminal, source the setup files from inside ros2_ws again, and then start the listener node:

```bash
ros2 run cpp_pubsub listener
```
The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

```bash
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```
Enter `Ctrl+C` in each terminal to stop the nodes from spinning.
### Summary
You created two nodes to publish and subscribe to data over a topic. Before compiling and running them, you added their dependencies and executables to the package configuration files.

## V Writing A Simple Publisher And Subscriber (Python)
### Background
In this tutorial, you will create nodes that pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

The code used in these examples can be found here.
### Prerequisites
In previous tutorials, you learned how to create a workspace and create a package.

A basic understanding of Python is recommended, but not entirely necessary.
### Tasks
#### 1 Create a package
Open a new terminal and source your ROS 2 installation so that `ros2` commands will work.

Navigate into the `ros2_ws` directory created in a previous tutorial.

Recall that packages should be created in the `src` directory, not the root of the workspace. So, navigate into `ros2_ws/src`, and run the package creation command:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```

Your terminal will return a message verifying the creation of your package `py_pubsub` and all its necessary files and folders.

#### 2 Write the publisher node
Navigate into `ros2_ws/src/py_pubsub/py_pubsub`. Recall that this directory is a Python package with the same name as the ROS 2 package it’s nested in.

Download the example talker code by entering the following command:

=== "LINUX"
    ```bash
    wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
    ```
=== "MACOS"
    ```bash
    wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
    ```
=== "WINDOWS"
    In a Windows command line prompt:
    ```bash
    curl -sk https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py -o publisher_member_function.py
    ```
    or with Powershell:
    ```bash
    curl https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py -o publisher_member_function.py
    ```

Now there will be a new file named `publisher_member_function.py` adjacent to `__init__.py`.

Open the file using your preferred text editor.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
##### 2.1 Examine the code
The first lines of code after the comments import rclpy so its Node class can be used.
```python
import rclpy
from rclpy.node import Node
```
The next statement imports the built-in string message type that the node uses to structure the data that it passes on the topic.

```python
from std_msgs.msg import String
```
These lines represent the node’s dependencies. Recall that dependencies have to be added to `package.xml`, which you’ll do in the next section.

Next, the `MinimalPublisher` class is created, which inherits from (or is a subclass of) `Node`.

```python
class MinimalPublisher(Node):
```
Following is the definition of the class’s constructor. `super().__init__` calls the `Node` class’s constructor and gives it your node name, in this case `minimal_publisher`.

`create_publisher `declares that the node publishes messages of type `String` (imported from the `std_msgs.msg` module), over a topic named `topic`, and that the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

Next, a timer is created with a callback to execute every 0.5 seconds. `self.i` is a counter used in the callback.

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

`timer_callback` creates a message with the counter value appended, and publishes it to the console with `get_logger().info`.

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```
Lastly, the main function is defined.

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```
First the `rclpy` library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.

##### 2.2 Add dependencies
Navigate one level back to the `ros2_ws/src/py_pubsub` directory, where the `setup.py`, `setup.cfg`, and `package.xml` files have been created for you.

Open `package.xml` with your text editor.

As mentioned in the previous tutorial, make sure to fill in the <description>, <maintainer> and <license> tags:

```xml
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```
After the lines above, add the following dependencies corresponding to your node’s import statements:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
This declares the package needs `rclpy` and `std_msgs` when its code is executed.

Make sure to save the file.

##### 2.3 Add an entry point
Open the `setup.py` file. Again, match the `maintainer`, `maintainer_email`, `description` and `license` fields to your `package.xml`:

```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
Add the following line within the `console_scripts` brackets of the `entry_points` field:

```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
Don’t forget to save.

##### 2.4 Check setup.cfg
The contents of the `setup.cfg` file should be correctly populated automatically, like so:

```cfg
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
```
This is simply telling setuptools to put your executables in lib, because ros2 run will look for them there.

You could build your package now, source the local setup files, and run it, but let’s create the subscriber node first so you can see the full system at work.

#### 3 Write the subscriber node
Return to `ros2_ws/src/py_pubsub/py_pubsub` to create the next node. Enter the following code in your terminal:

=== "LINUX"
    ```bash
    wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
    ```
=== "MACOS"
    ```bash
    wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
    ```
=== "WINDOWS"
    In a Windows command line prompt:
    ```bash
    curl -sk https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py -o subscriber_member_function.py
    ```
    or with Powershell:
    ```bash
    curl https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py -o subscriber_member_function.py
    ```

##### 3.1 Examine the code

Open the `subscriber_member_function.py` with your text editor.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from the topics tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

```python
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```
The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

The callback definition simply prints an info message to the console, along with the data it received. Recall that the publisher defines `msg.data = 'Hello World: %d' % self.i`

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

The `main` definition is almost exactly the same, replacing the creation and spinning of the publisher with the subscriber.

```python
minimal_subscriber = MinimalSubscriber()

rclpy.spin(minimal_subscriber)
```

Since this node has the same dependencies as the publisher, there’s nothing new to add to `package.xml`. The `setup.cfg` file can also remain untouched.


##### 3.2 Add an entry point
Reopen `setup.py` and add the entry point for the subscriber node below the publisher’s entry point. The `entry_points` field should now look like this:

```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```
Make sure to save the file, and then your pub/sub system should be ready.


#### 4 Build and run

You likely already have the `rclpy` and `std_msgs` packages installed as part of your ROS 2 system. It’s good practice to run `rosdep`in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

=== "LINUX"
    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    ```
=== "MACOS"
    rosdep only runs on Linux, so you can skip ahead to next step.

=== "WINDOWS"
    rosdep only runs on Linux, so you can skip ahead to next step.

Still in the root of your workspace, ros2_ws, build your new package:

=== "LINUX"
    ```bash
    colcon build --packages-select py_pubsub
    ```
=== "MACOS"
    ```bash
    colcon build --packages-select py_pubsub
    ```
=== "WINDOWS"
    ```bash
    colcon build --merge-install --packages-select py_pubsub
    ```
Now run the talker node:

```bash
ros2 run py_pubsub talker
```
The terminal should start publishing info messages every 0.5 seconds, like so:

```bash
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
...
```
Open another terminal, source the setup files from inside ros2_ws again, and then start the listener node:

```bash
ros2 run py_pubsub listener
```

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

```bash
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
...
```
Enter `Ctrl+C` in each terminal to stop the nodes from spinning.

### Summary

You created two nodes to publish and subscribe to data over a topic. Before running them, you added their dependencies and entry points to the package configuration files.





## VI Writing A Simple Service And Client (C++)



## VII Writing A Simple Service And Client (Python)


## VIII Creating Custom Msg and Srv Files


## IX Implementing Custom Interfaces


## X Using Parameters In A Class (C++)


## XI Using Parameters In A Class (Python)


## XII Using ros2doctor To Identify Issues


## XIII Creating And Using Plugins
# 动手学ROS2
1.为什么要学习ROS2？
2.学了本课程能够做什么？
ROS2强大之处在于其生态系统，基于ROS2的软件库和工具集，针对不同种类的机器人和应用场景，衍生出了一系列机器人框架，用于如导航，规划，协同。
3.本教程适合学习对象有哪些？
4.课程规划
5.学习资源

## CH1 ROS2介绍与安装

### 章节导读
基础篇-Linux
入门篇-ROS2介绍安装
进阶篇-架构与中间件

### 基础篇-Linux
#### 1.Linux与Ubuntu系统介绍
##### 1.Linux是什么
这里需要了解下什么是操作系统，以及Linux内核是什么？

操作系统：管理计算机硬件与软件资源的计算机程序。

内核：内核是驱动硬件的程序。

基于硬件的第一层软件扩充，提供操作系统的最基本的功能，是操作系统工作的基础，它负责管理系统的进程、内存、设备驱动程序、文件和网络系统，决定着系统的性能和稳定性。

基于Linux内核衍生出了很多Linux系操作系统，Ubuntu就是其中之一。

##### 2.Ubuntu是什么

##### 3.Ubuntu系统版本

##### 4. CPU架构

你需要知道的常见架构有：

- amd64
- arm
- aarch64
- x86/i386(不常用)
根据电脑使用的CPU架构不同，你安装Ubuntu系统时应该选择对应的Ubuntu安装镜像包。

同时注意：不同架构的不同操作系统的软件安装包也是不兼容的。

##### 5. Ubuntu权限管理

- Linux 系统中的 超级用户 root 账号通常 用于系统的维护和管理，对操作系统的所有资源 具有所有访问权限
- sudo 命令用来以其他身份来执行命令，预设的身份为 root，所以我们可以使用sudo + 命令来提升操作权限
- chmod 命令可以用于修改文件权限

##### 6. Ubuntu如何安装软件
你需要知道的是：

- 使用apt从服务器下载安装，你需要提前添加服务器地址和服务器的秘钥，这一步就叫添加源或者换源
- 使用源码进行编译安装，你需要下载源码和源码的各种依赖，之后编译出程序拷贝到系统中
- 为什么可以使用apt安装？其实就是软件开发者在自己电脑上编译好程序，把程序打包上传到服务器，你就可以从服务器下载安装了

#### 2.在虚拟机中安装Ubuntu系统

##### 1.下载Ubuntu系统镜像
!!! tip

    最好是LTS版本，因为LTS版本的软件库比较稳定，不会出现软件库中的软件版本不兼容的问题。
##### 2.安装虚拟机软件
!!! tip
    VMware
##### 3.安装Ubuntu系统

#### 3.玩转Ubuntu之常用命令

##### 1.学会打开终端
!!! tip
    
    Ctrl+Alt+T
##### 2.学会常用命令
ls
cd
pwd
mkdir
rmdir
touch
rm
cp
mv
cat
more
less
head
tail

#### 4.玩转Ubuntu之编程工具

!!! info
    做机器人最常用的两门语言就是C++和Python，同时这两门语言也是编程语言流行度排行榜数一数二的。


#### 5.玩转Ubuntu之常用软件

!!! info
    VSCode

### 入门篇-ROS2介绍安装

#### 1.ROS2前世今生
要说ROS2，那就不得不提起ROS，ROS就是传说中的机器人操作系统，英文全称（Robot Operating System），但ROS本身并不是一个操作系统，而是可以安装在现在已有的操作系统上（Linux、Windows、Mac）上的软件库和工具集。

ROS出生于2007年，ROS的出现解决了机器人各个组件之间的通信问题，同时基于ROS的完善的通信机制，越来越多的优秀的机器人算法集成到了ROS中来。

现在的ROS功能已经变得非常的丰富和强大。但随着对ROS功能上要求越来越多，一些原始的架构和设计不能够满足目前的使用需求，这也是ROS2出现的原因。

ROS2继承了ROS原有的优秀之处，同时又带来了很多新的功能,ROS2相对于ROS更加的强大。

##### 1.1 ROS为什么会出现
ROS的设计目的是：简化在各种机器人平台上创建复杂而强大的机器人行为的任务即不重复造轮子

在ROS没有出现之前，做一个机器人是非常复杂的一件事情，因为一个机器人需要涉及到多个部分，而且这些部分之间还要进行通信。

例如设计一个轮式移动机器人，我们对其进行拆解。可以分为感知、决策、控制三个部分。


感知部分有：激光雷达、深度相机、IMU、里程计、碰撞感知、建图
决策部分有：路径规划（navigation）算法、定位算法
控制部分有：轮子驱动
机器人复杂之处就在于此，如果想要整个机器人可以跑起来，那么必须要有一个东西将上面的几个部分合理的连接到一起，这个东西就是ROS。

ROS的作用就像我们的身体的神经系统一样，通过神经系统将我们身体的各个部分接入大脑。

##### 1.2 为什么还要有ROS2

2007年ROS开发人员设计和制作ROS时，当时只想着简化机器人的开发，并没有想到过今天那么多的功能需求，比如商业化要求的稳定性、生命周期管理、多机协同、数据加密等。就像小鱼建房子时没想要未来会用全自动洗衣机一样~

ROS发展的后面的几年里，机器人对ROS的功能要求越来越多，ROS开发人员只能在原有的ROS上修修补补。

随着ROS不断的添加新功能，ROS变得越来越臃肿，祖传代码也越来越多。ROS开发人员发现在原有的ROS架构上修修补补十分消耗头发，于是像小鱼决定把房子推倒重建一样，ROS官方也重新设计制作了ROS2。

##### 1.3 ROS2介绍

#### 2.ROS与ROS2的对比

##### 2.1 ROS问题举例
上节课说到ROS的设计目标是简化机器人的开发，如何简化呢？ROS为此设计了一整套通信机制（话题、服务、参数、动作）。

通过这些通信机制，ROS实现了将机器人的各个组件给的连接起来，在设计这套通信机制的时候就设计了一个叫做Ros Master的东西，所有节点（可以理解为某一个组件，比如：激光雷达）的通信建立必须经过这个主节点。

一旦Ros Master主节点挂掉后，就会造成整个系统通信的异常,此时避障策略将会失效，如果机器人正在运行，碰到障碍物会径直装上去，机毁人亡！

ROS的不稳定这个问题在虽然对大家做机器人研究问题不大，但如果是想基于ROS做商业化机器人（比如无人驾驶汽车），就会造成非常严重的后果，小鱼在工作中可没为这个问题发愁

除了不稳定这个问题，ROS还有很多其他地方存在着问题：

通信基于TCP实现，实时性差、系统开销大
对Python3支持不友好，需要重新编译
消息机制不兼容
没有加密机制、安全性不高

##### 2.2 ROS2与ROS架构对比
所以在ROS2中，首当其冲的将ROS的主节点干掉了，这里放一张网上流传最广的ROS/ROS2架构图，接下来就会按照这篇架构图给大家讲解。

该图出自论文：Exploring the Performance of ROS2，论文在线阅读地址：https://www.researchgate.net/profile/Takuya-Azumi/publication/309128426_Exploring_the_performance_of_ROS2/links/5c908801299bf14e7e84ce61/Exploring-the-performance-of-ROS2.pdf

![image](./img/arch.png)

##### 2.3 ROS2新概念例举
可用Python编写的Launch文件
多机器人协同通信支持
支持安全加密通信
同一个进程支持多个节点、
支持Qos服务质量
支持节点生命周期管理
高效的进程间通信
##### 2.4 更详细的对比
https://zhuanlan.zhihu.com/p/423581728

#### 3.动手安装ROS2

##### 3.1 一键安装

##### 3.2 手动安装

- 第一步： ctrl + alt + T 打开终端

- 第二步：添加源

```bash
echo "deb [arch=$(dpkg --print-architecture)] https://repo.huaweicloud.com/ros2/ubuntu/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

- 第三步：添加秘钥

```bash
sudo apt install curl gnupg2 -y
curl -s https://gitee.com/ohhuo/rosdistro/raw/master/ros.asc | sudo apt-key add -
```

- 第四步：更新

```bash
sudo apt update
```

- 第五步：安装

```bash
sudo apt install ros-humble-desktop
```

- 第六步：安装额外依赖

```bash
sudo apt install python3-argcomplete -y
```

- 第七步：设置环境变量

```bash
source /opt/ros/humble/setup.bash
```

避免每次都要手动source，可以将上面的命令写入到~/.bashrc文件中

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

##### 3.3 出现问题可以这样卸载
    
```bash
sudo apt remove ros-humble-*
sudo apt autoremove
```

##### 3.4 ROS2到底装在哪里了
```bash
cd /opt/ros/humble/
ls
```

#### 4.ROS2初体验

##### 4.1 你说我听

- 第一步：打开终端
ctrl + alt + T

- 第二步：启动倾听者
```bash
ros2 run demo_nodes_py listener
```

- 第三步：启动新的终端
ctrl + alt + T

- 第四步：启动说话者
```bash
ros2 run demo_nodes_cpp talker
```
##### 4.2 涂鸦小乌龟

- 第一步：启动小乌龟
```bash
ros2 run turtlesim turtlesim_node
```

- 第二步：启动键盘控制
```bash
ros2 run turtlesim turtle_teleop_key
```

##### 4.3 RQT可视化

保持前两个游戏运行，打开新的终端，输入命令：

```bash
rqt
```

选择插件

比如Introspection/Node Graph

##### 4.4 总结

### 进阶篇-架构与中间件

#### 1.ROS2架构

##### 1.1 架构图

##### 1.2 操作系统层

##### 1.3 DDS实现层

##### 1.4 抽象DDS层-RMW

##### 1.5 ROS客户端库RCL

##### 1.6 应用层

#### 2.中间件DDS架构

##### 2.1 中间件

##### 2.2 DDS和ROS2架构

##### 2.3 DDS通讯模型

##### 2.4 DDS的优势与劣势

## CH2 ROS2第一个节点

## CH3 ROS2节点通讯之话题与服务

## CH4 ROS2节点通讯之参数与动作

## CH5 常用工具
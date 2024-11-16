# 边缘智能节点开发 

## 架构

```shell
+----------------------------------------+
| 应用层 (Application Layer)              | <-- 应用代码
+----------------------------------------+
| 系统服务层 (System Services Layer)      | <-- 操作系统 + 中间件
+----------------------------------------+
| 平台抽象层 (Platform Abstraction Layer) | <-- 启动代码 + 硬件抽象层 (寄存器级) + 板级支持包 (开发板级)
+----------------------------------------+
| 硬件层 (Hardware Layer)                 | <-- 主控单元 + 外设
+----------------------------------------+
    
```

!!! info 
    除了应用层内容与具体用途强绑定外，其他层次的内容都是可以通用的，因此硬件，平台抽象层，系统服务层的开发是可以通用的，可以在不同的应用场景中进行移植。分别放入不同的仓库中，方便管理和维护。

## 节点硬件

!!! note "硬件开发"
    为了实现边缘智能计算，我们开发了两种 MCU 节点，基于 STM32 和 ESP32。这些节点具有高性能边缘计算能力，可用于物联网、智能家居、智能城市等应用场景。当前的开发重心是ESP32。

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __MCU_NODE_STM32🎯🏆__

    ---

    基于 STM32 的 MCU IoT 节点，具有高性能边缘计算


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/MCU_NODE_STM32.git" target="_blank"> 传送门 </a>](#)

-   :simple-github:{ .lg .middle } __MCU_NODE_ESP32🎯🏆__

    ---

    基于 ESP32 的 MCU IoT 节点，具有高性能边缘计算


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/MCU_NODE_ESP32.git" target="_blank"> 传送门 </a>](#)

</div>

## 驱动开发

!!! note "驱动开发"
    驱动开发是边缘智能节点开发的重要组成部分。我们开发了多种驱动程序，包括传感器驱动、通信驱动、显示驱动等。这些驱动程序为边缘智能节点提供了丰富的外设支持，使得节点可以更好地适应不同的应用场景。

!!! tip "提示"
    该仓库内收录的驱动实际上已经包括在上面ESP32和STM32的节点仓库中，这里仅作为整理和收录，方便在其他设备和平台进行移植。

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __MCU_BSP🎯🏆__

    ---

    当前支持开发板：

    - STM32

    - ESP32


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/MCU_BSP.git" target="_blank"> 传送门 </a>](#)

</div>

## TinySHM 开发

!!! note "TinySHM 开发"
    TinySHM 是专门针对结构健康监测开发的边缘计算使能框架，包含多种模块和服务，以中间件形式集成在边缘智能节点软件中。TinySHM 为结构健康监测提供了丰富的边缘计算支持，使得节点可以更好地适应不同的结构健康监测应用场景。其具有以下特点：

    - 模块化设计，易于扩展
    - 跨平台支持，可移植性强，为不同平台预留区域供自定义实现
    - 支持高性能计算，提供多种高性能计算模块，特别是针对结构健康监测的高性能计算模块

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __TinySHM🎯🏆__

    ---

    当前支持平台：

    - STM32

    - ESP32


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/TinySHM.git" target="_blank"> 传送门 </a>](#)

</div>
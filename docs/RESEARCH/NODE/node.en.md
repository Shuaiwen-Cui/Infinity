# EDGE INTELLIGENCE NODE DEVELOPMENT 

## ARCHITECTURE

```shell
+------------------------------+
| Application Layer            | <-- Application Code
+------------------------------+
| System Services Layer        | <-- OS + Middleware
+------------------------------+
| Platform Abstraction Layer   | <-- Startup Code + HAL (register-level) + BSP (board-level)
+------------------------------+
| Hardware Layer               | <-- Main Control + Peripherals
+------------------------------+
```

## NODE HARDWARE

!!! note "Hardware Development"
    To achieve edge intelligence computing, we have developed two types of MCU nodes, based on STM32 and ESP32. These nodes have high-performance edge computing capabilities and can be used in IoT, smart home, smart city, and other application scenarios. The current development focus is on ESP32.

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __MCU_NODE_STM32__

    ---

    MCU IoT Node with High Performance Edge Computing, Based on STM32


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/MCU_NODE_STM32.git" target="_blank"> Portal </a>](#)

-   :simple-github:{ .lg .middle } __MCU_NODE_ESP32__

    ---

    MCU IoT Node with High Performance Edge Computing, Based on ESP32


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/MCU_NODE_ESP32.git" target="_blank"> Portal </a>](#)

</div>

## DRIVER DEVELOPMENT

!!! note "Driver Development"
    Driver development is an important part of edge intelligence node development. We have developed various drivers, including sensor drivers, communication drivers, display drivers, etc. These drivers provide rich peripheral support for edge intelligence nodes, enabling nodes to better adapt to different application scenarios.

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __MCU_BSP__

    ---

    Currently Supported Development Boards:

    - STM32

    - ESP32


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/MCU_BSP.git" target="_blank"> Portal </a>](#)


</div>

## TinySHM DEVELOPMENT

!!! note "TinySHM Development"
    TinySHM is an edge computing enablement framework developed specifically for structural health monitoring, containing various modules and services, integrated in the form of middleware in edge intelligence node software. TinySHM provides rich edge computing support for structural health monitoring, enabling nodes to better adapt to different structural health monitoring application scenarios. It has the following characteristics:

    - Modular design, easy to expand
    - Cross-platform support, strong portability, reserved areas for custom implementation on different platforms
    - Rich middleware services, including data acquisition, data processing, data storage, data transmission, etc.

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __TinySHM__

    ---

    Currently Supported Platforms:

    - STM32

    - ESP32


    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/TinySHM.git" target="_blank"> Portal </a>](#)


</div>
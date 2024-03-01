# MY SETUP FOR EMBEDDED SYSTEMS DEVELOPMENT

!!! note
    There are so many choices for embedded systems development. This is my setup, and it works for me. Hope this can help you too.

!!! tip
    **Too Long Don't Read**
    - KEIL is always a good choice for embedded systems development. But the UI is not as good as VSCode.
    - VSCode is definitely a trend for embedded systems development. In terms of the plugins for embedded systems development to replace KEIL: **EIDE** > **KEIL MDK 6** (Feb 2024)

## My Arsenal

!!! info
    I will constantly update my arsenal and try to find better toolchain for embedded systems development.

This section serves as the building block for embedded systems development. And the dev toolchain (the next section) is actually a combination of different tools.

### Project Initializer 

Conventionally, we initialize a project by manually adding all the files we need into the project and then configuring the project settings. This is a tedious and error-prone process. 

Nowadays, GUI-based project initialization tools are available. They can help us to create a project with all the necessary files and settings by using GUI with a few clicks.

#### STM32CubeMX

STM32CubeMX is such a GUI tool for project initialization. However, note that is only for STM32 microcontrollers.

<div class="grid cards" markdown>

-   :material-book:{ .lg .middle } __STM32CubeMX 🎯🏆__

    ---

    STM32CubeMX is a graphical tool that allows a very easy configuration of STM32 microcontrollers and microprocessors, as well as the generation of the corresponding initialization C code for the Arm® Cortex®-M core or a partial Linux® Device Tree for Arm® Cortex®-A core, through a step-by-step process.

    [:octicons-arrow-right-24: <a href="https://www.st.com/en/development-tools/stm32cubemx.html" target="_blank"> Portal </a>](#)

-   :material-file:{ .lg .middle } __STM32CubeMX Specialization🎯🏆__

    ---

    In Chinese

    [:octicons-arrow-right-24: <a href="https://www.zhihu.com/column/STM32CubeMX" target="_blank"> Portal </a>](#)

</div>

#### PlatformIO

PlatformIO is an open source ecosystem for IoT development. It supports 750+ embedded boards, 35+ development platforms, 20+ frameworks. It is a cross-platform IDE and a unified debugger. It is a full-featured IDE which is compatible with Arduino, ARM mbed, STM32, ESP8266, ESP32, and many other popular hardware.

<div class="grid cards" markdown>

-  :simple-platformio:{ .lg .middle } __PlatformIO IDE 🎯🏆__

    ---

    PlatformIO IDE is the next-generation integrated development environment for IoT. Cross-platform build system and library manager. Continuous and IDE integration. Arduino, ESP8266 and ARM mbed compatible. Ready for Cloud compiling. In fact, it is a VSCode extension.

    **NOTE:** 
    Pros: PlatformIO is trying to be a universal tool for embedded systems development. It is a good choice for beginners and experienced developers. It is also a good choice for those who want to develop IoT applications.
    Cons: PlatformIO is not as powerful as STM32CubeMX in terms of project initialization. It is not a good choice for those who want to develop applications for a specific microcontroller. Also, the supported microcontrollers are limited.

    [:octicons-arrow-right-24: <a href="https://platformio.org/platformio-ide" target="_blank"> Portal </a>](#)

</div>

### Editor

Almost every IDE support code editing, Keil MDK, Source Insight, Understand, IAR, and so on. However, I prefer to use Visual Studio Code.

#### Visual Studio Code
When it comes to editing, VSCode is always my first choice. It is a lightweight but powerful source code editor which runs on your desktop and is available for Windows, macOS and Linux. It comes with built-in support for JavaScript, TypeScript and Node.js and has a rich ecosystem of extensions for other languages (such as C++, C#, Java, Python, PHP, Go) and runtimes.

<div class="grid cards" markdown>

-  :material-microsoft-visual-studio-code:{ .lg .middle } __Visual Studio Code 🎯✅🏆__

    ---

    Visual Studio Code is a lightweight but powerful source code editor which runs on your desktop and is available for Windows, macOS and Linux. 

    [:octicons-arrow-right-24: <a href="https://code.visualstudio.com/" target="_blank"> Portal </a>](#)

</div>

Some great extension for embedded systems development in VSCode:

- PlatformIO 🏆
  
- Keil Assistant 
  
- Embedded IDE (EIDE) 🏆

#### PlatformIO
<div class="grid cards" markdown>

-  :simple-platformio:{ .lg .middle } __PlatformIO IDE 🎯🏆__

    ---

    PlatformIO IDE is the next-generation integrated development environment for IoT. Cross-platform build system and library manager. Continuous and IDE integration. Arduino, ESP8266 and ARM mbed compatible. Ready for Cloud compiling. In fact, it is a VSCode extension.

    [:octicons-arrow-right-24: <a href="https://platformio.org/platformio-ide" target="_blank"> Portal </a>](#)

</div>

#### Keil Assistant

<div class="grid cards" markdown>

-   :fontawesome-brands-bilibili:{ .lg .middle } __STM32CubeMX + KEIL MDK + VSCode + Keil Assistant🎯✅🏆__
    
    ---

    A complete guide to setting up an embedded system development environment using STM32CubeMX, KEIL MDK, and VSCode. (In Chinese)

    Essentially, only the code editor function of VSCode is used, and other functions are implemented through Keil.

    Note: Keil Assistant Extension stopped to update, the author turned to develop Embedded IDE (EIDE) for VSCode instead, refer to the next card.

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1re4y1H7nw?p=1&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)

</div>

#### Embedded IDE (EIDE)

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __Embedded IDE (EIDE)🎯🏆__
    
    ---

    VSCode + GCC + EIDE = all muc development tools.

    With this extension, we can use VSCode to replace Keil MDK and realize the development of embedded systems. Compilation, debugging, download, and other operations can be performed in VSCode.

    [:octicons-arrow-right-24: <a href="https://em-ide.com/" target="_blank"> Portal </a>](#)

-   :material-web:{ .lg .middle } __STM32CubeMX + VSCode + Embedded IDE🎯🏆__
    
    ---

    STM32CubeMX: Project initialization configuration
    VSCode: Code editor
    VSCode - Embedded IDE: Extension of VSCode to realize embedded development functions, such as compilation, download, debugging, etc.

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1nr4y1R7Jb/?spm_id_from=333.788.recommend_more_video.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)

</div>

Note that in this setup, the complier are imported from outside. In tutorial, the author used AC5 (ARMCC) and AC6 (ARMCLANG) as the complier. If you already installed Keil, you can use the complier from Keil.

#### KEIL 6 (VSCode Extension - Official Support)

(1) Official Website

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __Keil MDK 6__
    
    ---

    Keil MDK 6 is a complete software development environment for a wide range of Arm Cortex-M based microcontroller devices. MDK includes the µVision IDE/Debugger, Arm C/C++ Compiler, and essential middleware components. It's easy to learn and use, yet powerful enough for the most demanding embedded applications.

    [:octicons-arrow-right-24: <a href="https://www.keil.arm.com/community/" target="_blank"> Portal </a>](#)

</div>

(2) Documentation

<div class="grid cards" markdown>

-   :material-file:{ .lg .middle } __Keil MDK 6 - Documentation__
    
    ---
    
    [:octicons-arrow-right-24: <a href="https://developer.arm.com/documentation/108029/0000/Get-started-with-an-example-project" target="_blank"> Portal </a>](#)

</div>

(3) Tutorial

<div class="grid cards" markdown>

-   :fontawesome-brands-bilibili:{ .lg .middle } __Keil MDK 6 - Tutorial__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1Zp421R7Gr/?spm_id_from=333.337.search-card.all.click&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)

-   :material-file:{ .lg .middle } __Keil MDK 6 - Tutorial__
    
    ---
    
    [:octicons-arrow-right-24: <a href="https://sdutvincirobot.feishu.cn/docx/SJ9FdnLXwoR3cTx93eOc4lZfnzh" target="_blank"> Portal </a>](#)
</div>

### Compiler

There are open source and commercial compilers for embedded systems development. When it comes to daily use, the compliers are usually integrated into the IDEs. For example, Keil MDK uses ARMCC, IAR uses IAR Embedded Workbench, and STM32CubeIDE uses GCC. PlatformIO uses GCC too.

Note that, Keil Assistant use Keil for compling and downloading In the case of Embedded IDE, it supports many different external compilers.

### Debugger

The debugger is an important part of the toolchain. It is used to debug the program running on the target hardware. The debugger is usually integrated into the IDE. 

### Flasher

The flasher is used to flash the program into the target hardware. It is usually integrated into the IDE.

## ToolChain

The toolchain is a combination of different tools. It is used to develop embedded systems. The toolchain usually includes the following parts:

[Initializer *] + [Editor] + [Compiler] + [Debugger] + [Flasher]

### VSCode + KEIL MDK 6 (VSCode Extension - Official Support)🎯✅🏆

Initializer: STM32CubeMX
Editor: VSCode
Compiler: VSCode + KEIL MDK
Debugger: VSCode + KEIL MDK
Flasher: VSCode + KEIL MDK

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __Keil MDK 6 🎯✅🏆__
    
    ---

    Keil MDK 6 is a complete software development environment for a wide range of Arm Cortex-M based microcontroller devices. MDK includes the µVision IDE/Debugger, Arm C/C++ Compiler, and essential middleware components. It's easy to learn and use, yet powerful enough for the most demanding embedded applications.

    [:octicons-arrow-right-24: <a href="https://www.keil.arm.com/community/" target="_blank"> Portal </a>](#)


-   :material-file:{ .lg .middle } __Keil MDK 6 - Documentation__
    
    ---
    
    [:octicons-arrow-right-24: <a href="https://developer.arm.com/documentation/108029/0000/Get-started-with-an-example-project" target="_blank"> Portal </a>](#)

-   :fontawesome-brands-bilibili:{ .lg .middle } __Keil MDK 6 - Tutorial✅__
    
    ---
    
    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1Zp421R7Gr/?spm_id_from=333.337.search-card.all.click&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)


-   :material-file:{ .lg .middle } __Keil MDK 6 - Tutorial__
    
    ---

    [:octicons-arrow-right-24: <a href="https://sdutvincirobot.feishu.cn/docx/SJ9FdnLXwoR3cTx93eOc4lZfnzh" target="_blank"> Portal </a>](#)

</div>

### VSCode + EIDE🎯✅🏆

Initializer: Manually / STM32CubeMX / PlatformIO / Others
Editor: VSCode + Embedded IDE (EIDE)
Compiler: Configurable: GCC / ARMCC / IAR / Others
Debugger: Configurable: OpenOCD / J-Link / ST-Link / Others
Flasher: Configurable

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __VSCode + EIDE🎯✅🏆__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1nr4y1R7Jb/?spm_id_from=333.788.recommend_more_video.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)

</div>

### STM32CubeMX + VSCode/Keil Assistant + KEIL MDK🎯✅🏆

Initializer: STM32CubeMX
Editor: VSCode + Keil Assistant
Compiler: KEIL MDK
Debugger: KEIL MDK
Flasher: KEIL MDK

<div class="grid cards" markdown>

-   :fontawesome-brands-bilibili:{ .lg .middle } __STM32CubeMX + KEIL MDK + VSCode + Keil Assistant🎯✅🏆__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV19V411g7gD/?spm_id_from=333.999.0.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)

</div>

!!! tip
    Note that, in this solution, the Keil Assistant relies on Keil to function. Some functions can not be down in VSCode.

### HAL-makefile + Arm-GCC + VSCode + OpenOCD + ST-Link

Initializer: Manually / STM32CubeMX / PlatformIO / Others
Editor: VSCode
Compiler: Arm-GCC
Debugger: OpenOCD
Flasher: ST-Link

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __HAL-makefile + Arm-GCC + VSCode + OpenOCD + ST-Link__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1Hi4y1r7b3/?spm_id_from=333.999.0.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)

</div>

### Linux + Makefile + GCC + OpenOCD + DAP-Link

All Open Source.
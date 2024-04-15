# 我的嵌入式开发工具包

!!! note
    嵌入式开发工具的选择过于庞杂，如果你只是想要找一套切实可行的开发套件方案，那么这里的方案可能会对你有所帮助。

!!! tip
    **太长不看版**
    - Keil 是久经市场检验的稳定选择，但是就是界面太古老了。
    - VSCode是编程开发者的首选，目前已经有的几个用于取代keil的插件来说，个人感觉 **EIDE** 比**KEIL MDK 6** 更加好用。

## 我的工具箱

!!! info
    我会不断更新我的武器库，并尝试寻找更好的嵌入式开发工具链。

这一节是嵌入式系统开发的基石。开发工具链（下一节）实际上是不同工具的组合。

### 项目初始化工具

传统上，我们通过手动添加所有需要的文件到项目中，然后配置项目设置来初始化一个项目。这是一个繁琐且容易出错的过程。

现在，有基于图形界面的项目初始化工具。它们可以帮助我们通过几次点击使用图形界面创建一个带有所有必要文件和设置的项目。

#### STM32CubeMX

STM32CubeMX 就是这样一个项目初始化的图形界面工具。但是请注意，它只适用于 STM32 微控制器。

<div class="grid cards" markdown>

-   :material-book:{ .lg .middle } __STM32CubeMX 🎯🏆__

    ---

    STM32CubeMX 是一个图形化工具，它允许非常容易地配置 STM32 微控制器和微处理器，以及通过一个逐步过程为 Arm® Cortex®-M 核生成相应的初始化 C 代码或为 Arm® Cortex®-A 核生成部分 Linux® 设备树。

    [:octicons-arrow-right-24: <a href="https://www.st.com/en/development-tools/stm32cubemx.html" target="_blank"> 传送门 </a>](#)

-   :material-file:{ .lg .middle } __STM32CubeMX 专题🎯🏆__

    ---

    中文

    [:octicons-arrow-right-24: <a href="https://www.zhihu.com/column/STM32CubeMX" target="_blank"> 传送门 </a>](#)

</div>

#### PlatformIO

PlatformIO 是一个面向物联网开发的开源生态系统。它支持 750+ 嵌入式板，35+ 开发平台，20+ 框架。它是一个跨平台的 IDE 和统一的调试器。它是一个功能齐全的 IDE，与 Arduino、ARM mbed、STM32、ESP8266、ESP32 和许多其他流行的硬件兼容。

<div class="grid cards" markdown>

-  :simple-platformio:{ .lg .middle } __PlatformIO IDE 🎯🏆__

    ---

    PlatformIO IDE 是面向物联网的下一代集成开发环境。跨平台构建系统和库管理器。连续和 IDE 集成。兼容 Arduino、ESP8266 和 ARM mbed。准备好云编译。实际上，它是一个 VSCode 扩展。

    **注意：**
    优点：PlatformIO 正在努力成为嵌入式系统开发的通用工具。它是初学者和有经验的开发人员的不错选择。它也是那些想要开发物联网应用的人的不错选择。
    缺点：PlatformIO 在项目初始化方面不如 STM32CubeMX 强大。它不适合那些想要为特定微控制器开发应用的人。此外，支持的微控制器有限。

    [:octicons-arrow-right-24: <a href="https://platformio.org/platformio-ide" target="_blank"> 传送门 </a>](#)

</div>

### 编辑器

几乎所有的嵌入式开发工具都有自己的编辑器，Keil MDK, Source Insight, Understand, IAR, 等等。但是，我更喜欢使用 Visual Studio Code。

#### Visual Studio Code

在编辑方面，VSCode 总是我的首选。它是一个轻量级但功能强大的源代码编辑器，可在桌面上运行，并适用于 Windows、macOS 和 Linux。它内置对 JavaScript、TypeScript 和 Node.js 的支持，并具有丰富的扩展生态系统，可用于其他语言（如 C++、C#、Java、Python、PHP、Go）和运行时。

<div class="grid cards" markdown>

-  :material-microsoft-visual-studio-code:{ .lg .middle } __Visual Studio Code 🎯✅🏆__

    ---

    Visual Studio Code 是一个轻量级但功能强大的源代码编辑器，可在桌面上运行，并适用于 Windows、macOS 和 Linux。

    [:octicons-arrow-right-24: <a href="https://code.visualstudio.com/" target="_blank"> 传送门 </a>](#)

</div>

VSCode中用于嵌入式开发的重要插件：

- PlatformIO🏆

- Keil Assist

- Embedded IDE (EIDE)🏆

#### PlatformIO

<div class="grid cards" markdown>

-  :simple-platformio:{ .lg .middle } __PlatformIO IDE 🎯🏆__

    ---

    PlatformIO IDE 是面向物联网的下一代集成开发环境。跨平台构建系统和库管理器。连续和 IDE 集成。兼容 Arduino、ESP8266 和 ARM mbed。准备好云编译。实际上，它是一个 VSCode 扩展。

    [:octicons-arrow-right-24: <a href="https://platformio.org/platformio-ide" target="_blank"> 传送门 </a>](#)

</div>

#### Keil Assistant

<div class="grid cards" markdown>

-   :fontawesome-brands-bilibili:{ .lg .middle } __STM32CubeMX + KEIL MDK + VSCode + Keil Assistant🎯✅🏆__

    ---

    使用 STM32CubeMX、KEIL MDK 和 VSCode 设置嵌入式系统开发环境的完整指南。

    本质上，只是使用了VSCode作为代码编辑器的功能，其他的功能都是通过Keil来实现的。

    注意: Keil Assistant 扩展停止更新，作者转而开发VSCode的嵌入式IDE (EIDE)，请参考下一张卡片。

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1re4y1H7nw?p=1&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

</div>

#### Embedded IDE (EIDE)

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __Embedded IDE (EIDE)🎯🏆__
    
    ---

    VSCode + GCC + EIDE = 任意单片机开发工具。

    使用这个扩展，我们可以使用VSCode取代Keil MDK，实现嵌入式系的开发。可以在VSCode中进行编译、调试、下载等操作。

    [:octicons-arrow-right-24: <a href="https://em-ide.com" target="_blank"> 传送门 </a>](#)

-   :material-web:{ .lg .middle } __STM32CubeMX + VSCode + Embedded IDE🎯🏆__

    ---

    STM32CubeMX: 项目初始化配置
    VSCode: 代码编辑器
    VSCode - Embedded IDE: 扩展VSCode实现嵌入式开发功能，如编译，下载，调试等。

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1nr4y1R7Jb/?spm_id_from=333.788.recommend_more_video.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

</div>

注意，在此设置中，编译器是从外部导入的。在教程中，作者使用了AC5 (ARMCC)和AC6 (ARMCLANG)作为编译器。如果您已经安装了Keil，您可以使用Keil中的编译器。

#### KEIL 6 (VSCode 插件)

(1) 官方网站

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __Keil MDK 6__
    
    ---

    Keil MDK 6 是一款为广泛的 Arm Cortex-M 微控制器设备提供的完整软件开发环境。MDK 包括 µVision IDE/Debugger、Arm C/C++ 编译器和必要的中间件组件。它易于学习和使用，但足够强大，可以满足最苛刻的嵌入式应用需求。

    [:octicons-arrow-right-24: <a href="https://www.keil.arm.com/community/" target="_blank"> 传送门 </a>](#)

</div>

(2) 文档

<div class="grid cards" markdown>

-   :material-file:{ .lg .middle } __Keil MDK 6 - Documentation__
    
    ---

    [:octicons-arrow-right-24: <a href="https://developer.arm.com/documentation/108029/0000/Get-started-with-an-example-project" target="_blank"> 传送门 </a>](#)

</div>

(3) 教程

<div class="grid cards" markdown>

-   :material-file:{ .lg .middle } __Keil MDK 6 - Tutorial__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1Zp421R7Gr/?spm_id_from=333.337.search-card.all.click&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

-   :fontawesome-brands-bilibili:{ .lg .middle } __Keil MDK 6 - Tutorial__
    
    ---
    
    [:octicons-arrow-right-24: <a href="https://sdutvincirobot.feishu.cn/docx/SJ9FdnLXwoR3cTx93eOc4lZfnzh" target="_blank"> 传送门 </a>](#)

</div>

### 编译器

嵌入式系统开发有开源和商业编译器。在日常使用中，编译器通常集成到 IDE 中。例如，Keil MDK 使用 ARMCC，IAR 使用 IAR Embedded Workbench，STM32CubeIDE 使用 GCC。PlatformIO 也使用 GCC。

请注意，Keil Assistant使用Keil进行编译和下载。在嵌入式IDE的情况下，它支持许多不同的外部编译器。

### 调试器

调试器是工具链的重要组成部分。它用于调试在目标硬件上运行的程序。调试器通常集成到IDE中。

### 下载器

下载器是用于将程序下载到目标硬件的工具。它通常集成到IDE中。

## 工具链

工具链是一系列工具的组合。它们通常是项目初始化工具、编辑器、编译器、调试器和下载器的组合。

[Initializer *] + [Editor] + [Compiler] + [Debugger] + [Flasher]

### VSCode + KEIL MDK 6 (VSCode 插件 - 官方)🎯✅🏆

初始化工具：手动 / STM32CubeMX / PlatformIO / 其他
编辑器：VSCode
编译器：VSCode + KEIL MDK
调试器：VSCode + KEIL MDK
下载器：VSCode + KEIL MDK

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __Keil MDK 6 🎯✅🏆__
    
    ---

    Keil MDK 6 is a complete software development environment for a wide range of Arm Cortex-M based microcontroller devices. MDK includes the µVision IDE/Debugger, Arm C/C++ Compiler, and essential middleware components. It's easy to learn and use, yet powerful enough for the most demanding embedded applications.

    [:octicons-arrow-right-24: <a href="https://www.keil.arm.com/community/" target="_blank"> 传送门 </a>](#)

-   :material-file:{ .lg .middle } __Keil MDK 6 - 官方文档__
    
    ---
    
    [:octicons-arrow-right-24: <a href="https://developer.arm.com/documentation/108029/0000/Get-started-with-an-example-project" target="_blank"> Portal </a>](#)

-   :fontawesome-brands-bilibili:{ .lg .middle } __Keil MDK 6 - 教程✅__
    
    ---
    
    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1Zp421R7Gr/?spm_id_from=333.337.search-card.all.click&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> Portal </a>](#)


-   :material-file:{ .lg .middle } __Keil MDK 6 - 教程配套文档__
    
    ---

    [:octicons-arrow-right-24: <a href="https://sdutvincirobot.feishu.cn/docx/SJ9FdnLXwoR3cTx93eOc4lZfnzh" target="_blank"> Portal </a>](#)
</div>

### VSCode + EIDE🎯✅🏆

初始化工具：手动 / STM32CubeMX / PlatformIO / 其他
编辑器：VSCode + Embedded IDE (EIDE)
编译器：可配置：GCC / ARMCC / IAR / 其他
调试器：可配置：OpenOCD / J-Link / ST-Link / 其他
下载器：可配置

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __VSCode + EIDE🎯✅🏆__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1nr4y1R7Jb/?spm_id_from=333.788.recommend_more_video.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

</div>

### STM32CubeMX + VSCode/Keil Assistant + KEIL MDK🎯✅🏆

初始化工具：STM32CubeMX
编辑器：VSCode + Keil Assistant
编译器：KEIL MDK
调试器：KEIL MDK
下载器：KEIL MDK

<div class="grid cards" markdown>

-   :fontawesome-brands-bilibili:{ .lg .middle } __STM32CubeMX + KEIL MDK + VSCode + Keil Assistant🎯✅🏆__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV19V411g7gD/?spm_id_from=333.999.0.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

</div>

!!! tip
    注意，在这个方案中，Keil Assistant 依赖Keil来发挥作用，有些功能在VSCode中是无法实现的。

### HAL-makefile + Arm-GCC + VSCode + OpenOCD + ST-Link

初始化工具：手动 / STM32CubeMX / PlatformIO / 其他
编辑器：VSCode
编译器：Arm-GCC
调试器：OpenOCD
下载器：ST-Link

<div class="grid cards" markdown>

-   :material-web:{ .lg .middle } __HAL-makefile + Arm-GCC + VSCode + OpenOCD + ST-Link__
    
    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1Hi4y1r7b3/?spm_id_from=333.999.0.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

</div>

### Linux + Makefile + GCC + OpenOCD + DAP-Link

完全开源的方案。
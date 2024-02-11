# Make 系列

## 概览

对于 C 和 C++ 编程, 当工程量逐渐增大，代码量逐渐增多时，我们需要一种自动化的编译工具来帮助我们管理代码。Make 是一种非常流行的自动化编译工具，它可以帮助我们自动化编译代码，管理代码的依赖关系，以及自动化执行测试等。与Make对应的文件是Makefile，它是一个文本文件，其中包含了一系列的规则，用于指定如何编译和链接代码。

当代码量进一步增大，且需要跨平台的时候，我们可能需要使用更加高级的自动化编译工具，比如CMake。CMake 是一个跨平台的自动化编译工具，它可以帮助我们生成不同平台下的编译工程。与CMake对应的文件是CMakeLists.txt，它是一个文本文件，其中包含了一系列的规则，用于指定如何编译和链接代码。

简单理解cmake与make的关系就是，cmake根据CMakeLists.txt生成Makefile，然后再使用make来编译代码。

参考文章：

<div class="grid cards" markdown>

-   :material-book:{ .lg .middle } __5分钟理解make/makefile/cmake/nmake🎯✅__

    ---

    CMake/Makefile/Make/NMake 理解

    [:octicons-arrow-right-24: <a href="https://zhuanlan.zhihu.com/p/111110992" target="_blank"> 传送门 </a>](#)

</div>

## GNU Make

<div class="grid cards" markdown>

-   :material-book:{ .lg .middle } __GNU Make 官方指南__

    ---

    [:octicons-arrow-right-24: <a href="https://www.gnu.org/software/make/" target="_blank"> 传送门 </a>](#)

</div>

## CMake

<div class="grid cards" markdown>

-   :material-book:{ .lg .middle } __CMake 官方指南__

    ---

    [:octicons-arrow-right-24: <a href="https://cmake.org/" target="_blank"> 传送门 </a>](#)

</div>


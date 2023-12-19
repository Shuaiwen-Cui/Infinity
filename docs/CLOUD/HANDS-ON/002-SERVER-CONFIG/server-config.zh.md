# 配置服务器

## 通用性配置

### Docker

#### 什么是 Docker？

Docker 是一个开源的应用容器引擎，基于 Go 语言 并遵从 Apache2.0 协议开源。它可以让开发者打包他们的应用以及依赖包到一个可移植的镜像中，然后发布到任何流行的 Linux 或 Windows 机器上，也可以实现虚拟化。容器是完全使用沙箱机制，相互之间不会有任何接口。

#### 如何安装 Docker？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 Docker：

```bash
sudo apt-get install docker
```

### Git

#### 什么是 Git？

Git 是一个分布式版本控制系统，可以让你跟踪文件的变化，并协调多人和多机器上的文件工作。它主要用于软件开发中的源代码管理，但也可以用于跟踪任何文件的变化。

#### 如何安装 Git？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 Git：

```bash
sudo apt-get install git
```

### Nginx

#### 什么是 Nginx？

Nginx 是一个 Web 服务器，也可以用作反向代理、负载均衡器、邮件代理和 HTTP 缓存。它是免费开源的软件，根据 2 条款 BSD 许可证的条款发布。

#### 如何安装 Nginx？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 Nginx：

```bash
sudo apt-get install nginx
```

### Node.js

#### 什么是 Node.js？

Node.js 是一个开源的跨平台 JavaScript 运行时环境，用于在浏览器之外执行 JavaScript 代码。Node.js 让开发人员可以使用 JavaScript 来编写命令行工具和服务器端脚本，以在将页面发送到用户的 Web 浏览器之前在服务器端运行脚本来生成动态网页内容。

#### 如何安装 Node.js？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 Node.js：

```bash
sudo apt-get install nodejs
```

### Node Package Manager (npm)

#### 什么是 npm？

npm 是 JavaScript 编程语言的包管理器。它是 JavaScript 运行时环境 Node.js 的默认包管理器。它由一个命令行客户端（也称为 npm）和一个在线数据库组成，其中包含公共和付费的私有包，称为 npm 注册表。

#### 如何安装 npm？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 npm：

```bash
sudo apt-get install npm
```

### Python

#### 什么是 Python？

Python 是一种面向对象、解释型的计算机程序设计语言，由 Guido van Rossum 于 1989 年底发明，第一个公开发行版发行于 1991 年。Python 语法简洁清晰，特色之一是强制用空白符（如空格）作为语句缩进。

#### 如何安装 Python？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 Python：

```bash
sudo apt-get install python
```

### Python Package Manager (pip)

#### 什么是 pip？

pip 是 Python 编程语言的包管理器。它是 Python 运行时环境的默认包管理器。它由一个命令行客户端（也称为 pip）和一个在线数据库组成，其中包含公共和付费的私有包，称为 pip 注册表。

#### 如何安装 pip？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 pip：

```bash
sudo apt-get install pip
```

### Python Virtual Environment (venv)

#### 什么是 venv？

venv 是一个工具，可以让你创建独立的 Python 环境。它创建一个文件夹，其中包含使用 Python 项目所需的所有可执行文件。

#### 如何安装 venv？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 venv：

```bash
sudo apt-get install venv
```

## 专用性配置

### Anaconda

#### 什么是 Anaconda？

Anaconda 是一个用于科学计算的 Python 发行版，支持 Linux、Mac、Windows 系统，提供了包管理与环境管理的功能，可以很方便地解决多版本 Python 并存、切换以及各种第三方包安装问题。

#### 如何安装 Anaconda？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 Anaconda：

```bash
sudo apt-get install anaconda
```

### Anaconda Virtual Environment (conda)

#### 什么是 conda？

conda 是一个工具，可以让你创建独立的 Python 环境。它创建一个文件夹，其中包含使用 Python 项目所需的所有可执行文件。

#### 如何安装 conda？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 conda：

```bash
sudo apt-get install conda
```

### UWF (firewall)

#### 什么是 UWF？

UWF 是一个防火墙，可以让你保护你的服务器免受恶意攻击。

#### 如何安装 UWF？（以 Linux 为例）

在 Linux 上，你可以使用以下命令安装 UWF：

```bash
sudo apt-get install uwf
```

#### 如何使用 UWF 检查端口状态？（以 Linux 为例）

在 Linux 上，你可以使用以下命令使用 UWF 检查端口状态：

```bash
sudo uwf status
```

使用以下命令检查哪些端口是开放的：

```bash
sudo uwf status verbose
```

你也可以使用 netstat 检查端口状态，使用以下命令：

```bash
sudo netstat -tulpn
```

或者

```bash
sudo ss -tulpn
```

你也可以使用 iptables 检查端口状态，使用以下命令：

```bash
sudo iptables -L
```

#### 如何使用 UWF 启用端口？（以 Linux 为例）

首先，你需要在云服务提供商的防火墙中启用端口，然后你需要在 UWF 中启用端口。

在 Linux 上，你可以使用以下命令使用 UWF 启用端口：

```bash
sudo uwf allow <port>/<protocol>
```

或者更简单一点，不需要输入协议：

```bash
sudo uwf allow <port>
```

例如，端口 80 用于 http，这通常是 http 的默认端口，因此你可以使用以下命令：

```bash
sudo uwf allow 80/tcp 
```



# 远程控制

## SSH
SSH是一种协议，允许您通过终端连接到远程计算机并对其进行控制。

### 用法
- 步骤1 - 打开终端
- 步骤2 - 输入 `ssh 用户名@主机名`
    - `用户名` 是远程计算机的用户名
    - `主机名` 是远程计算机的主机名，可以是IP地址或域名

### 注意事项
- 如果要使用SSH连接到远程计算机，必须确保远程计算机安装并运行了SSH服务器！！！

#### Windows
- 安装并运行OpenSSH服务器
    - [🔗 在Windows上开始使用OpenSSH](https://learn.microsoft.com/zh-cn/windows-server/administration/openssh/openssh_install_firstuse?tabs=gui)
        - OpenSSH客户端 - 从Windows计算机控制其他计算机
        - OpenSSH服务器 - 从其他计算机控制Windows计算机
    - 确保两者都已安装
        - 如何做到？请参考上面的链接
    - 确保SSH服务器正在运行，并且可以在Windows启动时自动启动：
        - 步骤1 - 打开 `服务`
        - 步骤2 - 找到 `OpenSSH SSH Server` 和 `OpenSSH Authentication Agent`
        - 步骤3 - 右键单击它们，选择 `启动`
        - 步骤4 - 右键单击它们，选择 `属性`
        - 步骤5 - 在 `启动类型` 中选择 `自动`

#### Linux - Ubuntu

- step 1 - 安装openssh-server

```bash
sudo apt-get install openssh-server
```

- step 2 - 启用ssh服务器

```bash
sudo service ssh start
```

- step 3 - 查看ssh服务器是否正在运行

```bash
sudo service ssh status
```

- step 4 - 启用ssh服务器在系统启动时自动启动

```bash
sudo systemctl enable ssh
```

- step 5 - 查看ssh服务器是否正在运行（开机启动）

```bash
sudo systemctl is-enabled ssh
```

### 软件
一些软件可用于通过SSH连接到远程计算机，例如：
- Windows
    - [MobaXterm](https://mobaxterm.mobatek.net/)
        - 我在Windows上最喜欢的SSH客户端
    - [PuTTY](https://www.putty.org/)
        - Windows上最流行的SSH客户端
- MacOS
    - [Termius](https://termius.com/)
        - 我在MacOS上最喜欢的SSH客户端
- Linux
    - [OpenSSH](https://www.openssh.com/)
        - Linux上最流行的SSH客户端

## VNC
VNC是一种协议，允许您通过图形用户界面连接到远程计算机并对其进行控制。

### 用法
- 步骤1 - 打开VNC客户端
- 步骤2 - 输入 `主机名:端口`
    - `主机名` 是远程计算机的主机名，可以是IP地址或域名
    - `端口` 是远程计算机上VNC服务器的端口
- 步骤3 - 输入远程计算机上VNC服务器的密码
- 步骤4 - 通过图形用户界面控制远程计算机
- 步骤5 - 关闭VNC客户端

### 注意事项
- 如果要使用VNC连接到远程计算机，必须确保远程计算机安装并运行了VNC服务器！！！
- 如果不知道如何操作，请搜索相关信息。

#### Linux - Ubuntu
[how to enable VNC on Ubuntu LTS22](https://linuxstory.org/how-to-install-and-configure-vnc-on-ubuntu-22-04/)

### 软件
一些软件可用于通过VNC连接到远程计算机，例如：
- [RealVNC](https://www.realvnc.com/)
    - 我最喜欢的
- [TigerVNC](https://tigervnc.org/)

## 远程桌面软件
### 跨平台
- [RustDesk](https://rustdesk.com/)
    - 开源
    - 支持自建服务器
    - 界面友好
    - 自建服务器的安装和配置比较复杂，但是质量很好
    - 非私有服务器通道连接质量要差
    - 目前windows连接macos的好像不能控制，只能观看

- [ToDesk](https://www.todesk.com/)
    - 我最喜欢的！几乎可以做任何事情！
    - 强烈推荐！
    - 非常稳定，支持高分辨率
    - 个人使用免费
    - 为了更好的用户体验，付费
    - 对于跨国连接，最好购买适用的插件
    - 他们确实有一个Linux版本。我尝试在Linux上使用，但尚未成功。

- [Sunlogin](https://sunlogin.oray.com/)
    - 也许是中国最好的一个
    - 似乎不能在中国以外的地方使用

- [TeamViewer](https://www.teamviewer.com/)
    - 总是说我在商业用途中使用，然后在几分钟后就会断开连接
    - 我不喜欢它

- [AnyDesk](https://anydesk.com/)
    - 有时候它只是不起作用
    - 我不喜欢它

- [Chrome Remote Desktop](https://remotedesktop.google.com/)
    - 我尝试过这个，但尚未成功。

### Windows
- [RemoteDesktop]
    - 内置功能

### MacOS
- [Microsoft Remote Desktop](https://apps.apple.com/us/app/microsoft-remote-desktop/id1295203466?mt=12)
    - 这可以用来控制Windows
    - 非常稳定，支持高分辨率

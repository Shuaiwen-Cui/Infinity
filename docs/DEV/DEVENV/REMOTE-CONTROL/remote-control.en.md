# Remote Control

## SSH
SSH is a protocol that allows you to connect to a remote machine and control it via a terminal.

### USAGE
- STEP 1 - Open a terminal
- STEP 2 - Type `ssh username@hostname`
    - `username` is the username of the remote machine
    - `hostname` is the hostname of the remote machine, can be an IP address or a domain name

### NOTES
- If you want to use SSH to connect to a remote machine, you need to make sure that the remote machine has an SSH server installed and running!!!

#### Windows
- have OpenSSH Server installed, and running
    - [🔗 Get Started With OpenSSH On Windows](https://learn.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse?tabs=gui)
        - OpenSSH Client - control other computers from the Windows computer
        - OpenSSH Server - control the Windows computer from other computers
    - Ensure both of them are installed
        - How to do that? Follow the link above
    - Ensure the SSH server is running and can start to run automatically when windows starts up:
        - Step 1 - Open `Services`
        - Step 2 - Find `OpenSSH SSH Server` and `OpenSSH Authentication Agent`
        - Step 3 - Right Click on them and select `Start`
        - Step 4 - Right Click on them and select `Properties`
        - Step 5 - Select `Automatic` in `Startup type`

#### Linux - Ubuntu

- step 1 - install openssh-server

```bash
sudo apt-get install openssh-server
```

- step 2 - start ssh server

```bash
sudo service ssh start
```

- step 3 - check if ssh server is running

```bash
sudo service ssh status
```

- step 4 - enable ssh server to start automatically when system starts up

```bash
sudo systemctl enable ssh
```

- step 5 - check if ssh server is running

```bash
sudo systemctl is-enabled ssh
```


### SOFTWARES
Some softwares can be used to connect to a remote machine via SSH, such as:
- Windows
    - [MobaXterm](https://mobaxterm.mobatek.net/)
        - My favorite SSH client on Windows
    - [PuTTY](https://www.putty.org/)
        - Most popular SSH client on Windows
- MacOS
    - [Termius](https://termius.com/)
        - My favorite SSH client on MacOS
- Linux
    - [OpenSSH](https://www.openssh.com/)
        - Most popular SSH client on Linux

## VNC
VNC is a protocol that allows you to connect to a remote machine and control it via a GUI.

### USAGE
- STEP 1 - Open a VNC client
- STEP 2 - Type `hostname:port`
    - `hostname` is the hostname of the remote machine, can be an IP address or a domain name
    - `port` is the port of the VNC server on the remote machine
- STEP 3 - Enter the password of the VNC server on the remote machine
- STEP 4 - Control the remote machine via the GUI
- STEP 5 - Close the VNC client

### NOTES
- If you want to use VNC to connect to a remote machine, you need to make sure that the remote machine has a VNC server installed and running!!!
- If you don't know how to do that, please google it.

#### Ubuntu
[how to enable VNC on Ubuntu LTS22](https://linuxstory.org/how-to-install-and-configure-vnc-on-ubuntu-22-04/)

### SOFTWARES
Some softwares can be used to connect to a remote machine via VNC, such as:
- [RealVNC](https://www.realvnc.com/)
    - My favorite
- [TigerVNC](https://tigervnc.org/)

## SOFTWARE FOR REMOTE DESKTOP
### CrossPlatform

- [RustDesk](https://rustdesk.com/)
    - Open Source
    - Support self-hosted server
    - User friendly
    - The installation and configuration of self-hosted server is a little bit complicated, but   the quality is good
    - Public server is not stable
    - Windows to MacOS seems have some problems

- [ToDesk](https://www.todesk.com/)
    - My favorite one! All most can do everything!
    - Highly Recommended!
    - Pretty stable and features high resolution
    - Free for personal use
    - Paid for better user experience
    - For cross country country, better to buy the plugin for that
    - They do have a linux version. I have tried to use this on Linux, but not succeed yet. 

- [Sunlogin](https://sunlogin.oray.com/)
    - maybe the best one for China
    - seems cannot be used outside China
  
- [TeamViewer](https://www.teamviewer.com/)
    - it always says that I am using it for commercial use, and then it will disconnect me after a few minutes
    - I don't like it

- [AnyDesk](https://anydesk.com/)
    - sometimes it just not works
    - I don't like it

- [Chrome Remote Desktop](https://remotedesktop.google.com/)
    - I tried this one, but not succeed yet.

### Windows
- [RemoteDesktop]
    - built-in function

### MacOS
- [Microsoft Remote Desktop](https://apps.apple.com/us/app/microsoft-remote-desktop/id1295203466?mt=12)
    - This can be used to control a windows
    - Pretty stable and features high resolution


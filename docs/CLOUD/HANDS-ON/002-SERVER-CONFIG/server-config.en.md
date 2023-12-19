# Configure Your Server For Use

## Generic Configuration

### Docker

#### What is Docker?

Docker is a containerization platform that allows you to run applications in an isolated environment. It is a tool that can package an application and its dependencies in a virtual container that can run on any Linux server. This provides a lightweight, standalone, portable, and self-sufficient environment for applications to run.

#### How to Install Docker? (Linux as an example)

On Linux, you can install Docker using the following command:

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

### Git

#### What is Git?

Git is a distributed version control system that allows you to track changes to files and coordinate work on those files among multiple people and machines. It is primarily used for source code management in software development, but it can be used to keep track of changes in any files.

#### How to Install Git? (Linux as an example)

On Linux, you can install Git using the following command:

```bash
sudo apt-get install git
```

### Nginx

#### What is Nginx?

Nginx is a web server that can also be used as a reverse proxy, load balancer, mail proxy, and HTTP cache. It is free and open-source software, released under the terms of the 2-clause BSD license.

#### How to Install Nginx? (Linux as an example)

On Linux, you can install Nginx using the following command:

```bash
sudo apt-get install nginx
```

### Node.js

#### What is Node.js?

Node.js is an open-source, cross-platform, JavaScript runtime environment that executes JavaScript code outside of a browser. Node.js lets developers use JavaScript to write command line tools and for server-side scripting—running scripts server-side to produce dynamic web page content before the page is sent to the user's web browser.

#### How to Install Node.js? (Linux as an example)

On Linux, you can install Node.js using the following command:

```bash
sudo apt-get install nodejs
```

### Node Package Manager (npm)

#### What is npm?

npm is a package manager for the JavaScript programming language. It is the default package manager for the JavaScript runtime environment Node.js. It consists of a command line client, also called npm, and an online database of public and paid-for private packages, called the npm registry.

#### How to Install npm? (Linux as an example)

On Linux, you can install npm using the following command:

```bash
sudo apt-get install npm
```

### Python

#### What is Python?

Python is an interpreted, high-level, general-purpose programming language. Created by Guido van Rossum and first released in 1991, Python's design philosophy emphasizes code readability with its notable use of significant whitespace.

#### How to Install Python? (Linux as an example)

On Linux, you can install Python using the following command:

```bash
sudo apt-get install python3
```

### Python Package Manager (pip)

#### What is pip?

pip is the standard package manager for Python. It allows you to install and manage additional packages that are not part of the Python standard library.

#### How to Install pip? (Linux as an example)

On Linux, you can install pip using the following command:

```bash
sudo apt-get install python3-pip
```

### Python Virtual Environment (venv)

#### What is venv?

venv is a tool that allows you to create isolated Python environments. It creates a folder which contains all the necessary executables to use the packages that a Python project would need.

#### How to Install venv? (Linux as an example)

On Linux, you can install venv using the following command:

```bash
sudo apt-get install python3-venv
```
## Specific Configuration - For Computing and Data Science

### Anoconda

#### What is Anoconda?

Anaconda is a free and open-source distribution of the Python and R programming languages for scientific computing, that aims to simplify package management and deployment. Package versions are managed by the package management system conda.

#### How to Install Anoconda? (Linux as an example)

On Linux, you can install Anoconda using the following command:

```bash
sudo apt-get install anaconda
```

### Anoconda Virtual Environment (conda)

#### What is conda?

conda is an open-source package management system and environment management system that runs on Windows, macOS, and Linux. Conda quickly installs, runs, and updates packages and their dependencies. Conda easily creates, saves, loads, and switches between environments on your local computer. It was created for Python programs, but it can package and distribute software for any language.

#### How to Install conda? (Linux as an example)

On Linux, you can install conda using the following command:

```bash
sudo apt-get install conda
```

### UWF (firewall)

#### What is UWF?

UFW, or Uncomplicated Firewall, is an interface to iptables that is geared towards simplifying the process of configuring a firewall. While iptables is a solid and flexible tool, it can be difficult for beginners to learn how to use it to properly configure a firewall. If you're looking to get started securing your network, and you're not sure which tool to use, UFW may be the right choice for you.

#### How to Install UWF? (Linux as an example)

On Linux, you can install UWF using the following command:

```bash
sudo apt-get install ufw
```
Activate UWF using the following command:

```bash
sudo ufw enable
```

#### How to check the port status using UWF? (Linux as an example)

On Linux, you can check the port status using UWF using the following command:

```bash
sudo ufw status
```

check which ports are open using the following command:

```bash
sudo ufw status verbose
```

you can also use netstat to check the port status using the following command:

```bash
sudo netstat -tulpn
```

or
    
```bash
sudo ss -tulpn
```

you can also use iptables to check the port status using the following command:

```bash
sudo iptables -L
```

#### How to use uwf to enable ports? (Linux as an example)

First, you need to enable the ports in the cloud service provider's firewall, and then you need to enable the ports in the UWF.

On Linux, you can use UWF to enable ports using the following command:

```bash
sudo ufw allow <port>/<protocol>
```
or you can ignore the protocol for simplicity:

```bash
sudo ufw allow <port>
```

for example port 80 for http, this is usually the default port for http, so you can use the following command:

```bash
sudo ufw allow 80/tcp 
```


# Get Yourself A Cloud Server

## What Is A Cloud Server

A cloud server is a virtual server (rather than a physical server) running in a cloud computing environment. It is built, hosted and delivered via a cloud computing platform via the internet, and can be accessed remotely. They are also known as virtual servers.

## Cloud Server Providers

There are many cloud server providers, some of them are listed below:

- [Amazon Web Services](https://aws.amazon.com/)

- [Microsoft Azure](https://azure.microsoft.com/)

- [Google Cloud](https://cloud.google.com/)

- [Alibaba Cloud](https://www.alibabacloud.com/) [Personal Choice]

- [Tencent Cloud](https://intl.cloud.tencent.com/)

- [Huawei Cloud](https://www.huaweicloud.com/)

These providers provide similar services, pick one that suits you the best.

## How To Determine The Configuration Of The Cloud Server

There are some key factors that you need to consider when choosing the configuration of the cloud server:

- **CPU** - This determines the computing power of the cloud server.

- **Memory** - This determines the amount of memory of the cloud server.

- **Storage** - This determines the amount of storage of the cloud server.

- **Bandwidth** - This relates to the network speed of the cloud server.

- **Location** - This determines the location of the cloud server. The closer the location is to you, the faster the network speed is.

- **Price** - No need to explain.
  
- **Use Purpose** - The use purpose determines the bottom line of the configuration. For example, if you are going to use the cloud server to run a website, you need to make sure that the configuration is good enough to run the website smoothly.

## Explore The Cloud Server Provider's Website

Check what services the cloud server provider provides, and determine your own choice.

For me, for example, I picked ECS (Elastic Cloud Server) of Alibaba Cloud, which is a virtual server that can be used to host websites, run enterprise applications, or create development environments.

## Create An Account

Usually, the provider will require you to create an account before you can use their services. You can manage all your cloud servers in your account.

## Create An Instance
After payment of your cloud server, you can create an instance. An instance is a virtual server that can be used to host websites, run enterprise applications, or create development environments. You can create an instance on the cloud server provider's website. Usually, the provider will provide a wizard to help you create an instance.

After creating an instance, you will be able to see your instance in your account. You can access necessary information of your instance in your account, such as the IP address, the username and password, etc.

## Choose And Install Operating System

You can either determine the operating system of your instance when creating the instance, or you can install the operating system after creating the instance. Usually, the provider will provide a wizard to help you install the operating system. You can also install the operating system by yourself.

In general, there are two types of operating systems on the market for cloud servers, Windows and Linux. Most cloud server providers provide both Windows and Linux. You can choose the operating system that suits you the best.

- **Windows** - Windows is a series of operating systems developed by Microsoft. Windows is the most popular operating system in the world. Windows is easy to use, and it is suitable for beginners. However, Windows is not free, and it is not open source.

- **Linux** - Linux is a family of open-source Unix-like operating systems based on the Linux kernel, an operating system kernel first released on September 17, 1991, by Linus Torvalds. Linux is free, and it is open source. Linux is suitable for advanced users. There are many distributions of Linux, such as Ubuntu, CentOS, Debian, etc.

Most cases, for cloud server, you will choose Linux. Popular choices are Ubuntu and CentOS. For me, I chose Ubuntu.

- **Ubuntu** - Ubuntu is a free and open-source Linux distribution based on Debian. Ubuntu is officially released in three editions: Desktop, Server, and Core for Internet of things devices and robots. All the editions can run on the computer alone, or in a virtual machine. Ubuntu is a popular operating system for cloud servers.

- **CentOS** - CentOS is a Linux distribution that provides a free, community-supported computing platform functionally compatible with its upstream source, Red Hat Enterprise Linux (RHEL). CentOS is a popular operating system for cloud servers.

## Connect To The Cloud Server

After creating the instance, you can connect to the cloud server. Usually, the provider will provide a wizard to help you connect to the cloud server. You can also connect to the cloud server by yourself.

If you use Aliyun like me, you need to setup the password inside the instance management page, and then you can connect to the cloud server.

!!! tip
    For terminal tools, you can you the web-based SSH provided by the provider, or you can use third-party tools. On windows, you can use [PuTTY](https://www.putty.org/), or [MobaXterm](https://mobaxterm.mobatek.net/) (My Favorite!). On Mac, you can use third-party tools such as [Termius](https://termius.com/). They can help you remember the IP address, username and password of your cloud server, and you can connect to your cloud server with one click. Also, you can use them to transfer files between your local computer and your cloud server. Most of them are free or have free plans.

To connect to your cloud server, you need to know the IP address, username and password of your cloud server. You can find them in your account.

To use ssh to connect to your cloud server, you can use the following command:

```bash
ssh username@ip_address
```

## Install Necessary Softwares

After connecting to your cloud server, you can install necessary software. For example, you can install [Docker](https://www.docker.com/) to run containers, or you can install [Nginx](https://www.nginx.com/) to run a website.

## Upload Files To The Cloud Server

After connecting to your cloud server, you can upload files to your cloud server. For example, you can upload files to run a website.

!!! tip
    You can use third-party tools such as [WinSCP](https://winscp.net/eng/index.php) to upload files to your cloud server.

    Moreover, with Git and GitHub, you can clone your repository to your cloud server, and pull the latest changes from your repository to your cloud server.

## Use The Cloud Server

This part is up to you. You can use the cloud server to run a website, or use it as a broker for your IoT devices, or use it as a development environment, etc.

## The End

That's all for this article. Hope you enjoy it! 






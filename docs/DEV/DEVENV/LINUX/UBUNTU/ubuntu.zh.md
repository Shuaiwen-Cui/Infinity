# UBUNTU

!!! info
    Ubuntu 是一个基于 Debian 的开源操作系统，是一个 Linux 发行版。它是由南非企业家马克·沙特尔沃斯创建的，目前由 Canonical Ltd. 公司进行开发和维护。

## 安装中文输入法

<div class="grid cards" markdown>

-  :fontawesome-brands-ubuntu:{ .lg .middle } __Ubuntu 22.04 中文输入法支持__
    
    ---
    
    Ubuntu 22.04 中文输入法支持
    
    [:octicons-arrow-right-24: <a href="https://www.ubuntubuzz.com/2023/01/how-to-setup-ubuntu-computer-for-chinese-input-writing.html#google_vignette" target="_blank"> 传送门 </a>](#)

-   :fontawesome-brands-ubuntu:{ .lg .middle } __Ubuntu 22.04 中文简体拼音输入法支持__
    
    ---
    
    Ubuntu 22.04 中文简体拼音输入法支持
    
    [:octicons-arrow-right-24: <a href="https://askubuntu.com/questions/1408873/ubuntu-22-04-chinese-simplified-pinyin-input-support" target="_blank"> 传送门 </a>](#)

</div>

!!!  tip
    注意，有时候切换输入法只是针对当前窗口，如果需要全局切换，可以在设置中进行配置。

!!! tip
    从软件商城中下载的VSCode是个阉割版，有时候无法支持切换为中文输入法，此时应该写在VSCode并从官方渠道下载安装。

1. 删除阉割版VSCode
```bash
sudo snap remove code
```
2. 下载官方版VSCode
下载并存放到指定目录，然后安装
```bash
sudo dpkg -i code_1.86.2-1707854558_amd64.deb 
```
注意以上文件包请替换为你下载的文件包名称，这里只是举例。

# 个人仓库
该项目用来保存我的个人资源。

## 仓库结构
仓库结构与项目Infinity结构保持一致。

## 项目宿主

### GitHub Pages
[Github Pages](https://shuaiwen-cui.github.io/Warehouse/)

部署遵循常规github pages部署流程。

### 个人云服务器
[www.cuishuaiwen.com:7500](http://www.cuishuaiwen.com:7500/)

部署流程如下：

- 在云服务器中安装git。
```bash
sudo apt-get install git
```

- 切换到你想要存储仓库的位置，将仓库克隆到你的云服务器。
```bash
git clone https://github.com/Shuaiwen-Cui/Warehouse.git
```

- 在云服务器中安装nginx。
```bash
sudo apt-get install nginx
```

- 在云服务器的管理页面中，开启xxxx端口。

- 如果需要，你还需要在你的操作系统防火墙中开启xxxx端口。（先不要开启这一步，先试试不开启这一步）

- 配置nginx.conf文件，开启xxxx端口。
```bash
sudo vim /etc/nginx/sites-available/default
```

在文件中插入以下代码，并保存。
```bash
server {
    listen xxxx; # xxxx是你想要开启的端口
    server_name www.cuishuaiwen.com; # 你的域名或者其他
    root /home/ubuntu/Warehouse; # 你的仓库路径
    index index.html; # 你的index文件
}
```

- 重启nginx。
```bash
sudo service nginx restart
```
如果你的nginx已经注册为系统服务，你可以使用以下命令重启nginx。
```bash
sudo systemctl restart nginx
```

现在你可以访问你的网页了，或者在其他项目中使用链接来访问你的资源。注意，如果你的端口号不是80，你需要在链接中指定端口号。比如，我的端口号是7500，那么我的链接就是：http://www.cuishuaiwen.com:7500/。
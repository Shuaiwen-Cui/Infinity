# Warehouse
To store my resources.

## Warehouse Organization
The structure is consistent with the project Infinity structure.

## Project Hosts

### GitHub Pages
[Github Pages](https://shuaiwen-cui.github.io/Warehouse/)

The deployment follows common github pages deployment procedures.

### Personal Cloud Web Server
[www.cuishuaiwen.com:7500](http://www.cuishuaiwen.com:7500/)

Deployment follows the following steps:

1. Install git in your cloud server.

```bash
sudo apt-get install git
```

2. Change directory to the location you want to store the repo, clone the warehouse repository to your cloud server.

```bash
git clone https://github.com/Shuaiwen-Cui/Warehouse.git
```

3. Install nginx in your cloud server.

```bash
sudo apt-get install nginx
```

4. Enable the port xxxx in your cloud server, in the management page provided by your cloud service provider. 
   
5. If necessary, you also need to enable the port in your OS firewall. (try without this step first)

6. Configure the nginx.conf file to enable the port xxxx.

```bash
sudo vim /etc/nginx/sites-available/default
```
insert the following code into the file, and save it.

```bash
server {
    listen xxxx; # xxxx is the port you want to enable
    server_name www.cuishuaiwen.com; # your domain name or anything else
    root /home/ubuntu/Warehouse; # the path to your repo
    index index.html; # the index file
}
```

7. Restart nginx.

```bash
sudo service nginx restart
```

if you registered nginx as a service, you can also use the following command:

```bash
sudo systemctl restart nginx
```

1. Now you can visit the website, or use link to visit the file in this repo in other projects, for example, your personal tech blog. Note that, the port number is necessary if it is not 80 (default port). For example, my port number is 7500, then my link is: http://www.cuishuaiwen.com:7500/.



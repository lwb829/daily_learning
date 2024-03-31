# Docker学习笔记

推荐参考官方文档进行学习

再提供一个中文文档：[Docker—从入门到实践](https://yeasy.gitbook.io/docker_practice/)



## 1. Docker安装

Docker官方提供桌面版的Docker软件（wins/linux/mac）：[Docker Desktop](https://www.docker.com/)

在linux系统中，可以直接使用包管理工具，如ubuntu中的apt

- 在VSCODE中，非常推荐安装Docker的扩展，可以提供Dockerfile的语法检测、代码高亮、自动补全等

  也可以在菜单运行各种Docker命令，并且在左侧面板中也可以看到创建的所有镜像、容器等等

- 需要在安装Docker之后，然后在VSCODE中安装插件



### 1.1 安装Docker步骤

[docker配置官方教程](https://github.com/autowarefoundation/autoware_ai_documentation/wiki/docker-installation)

- 删除旧版或者不兼容版本的Docker

  ```
  $ sudo apt-get remove docker docker-engine docker.io
  ```

- 安装依赖

  ```
  $ sudo apt-get update
  $ sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
  ```

- 添加Docker官方的GPG密钥

  ```
  $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
  ```

- 密钥验证

  ```
  $ sudo apt-key fingerprint 0EBFCD88
  ```

- 添加Docker源到系统

  ```
  $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
  ```

- 安装Docker

  ```
  $ sudo apt-get update
  $ sudo apt-get install docker-ce
  ```

- 测试安装是否成功

  ```
  $ sudo docker run hello-world
  ```



### 1.2 安装nvidia-docker2

nvidia-docker2是对docker的封装，提供一些必要的组件可以很方便的在容器中使用GPU资源执行代码，nvidia-docker2共享了宿主机的CUDA Driver 通过命令行直接安装

```
$ sudo apt install nvidia-docker2
```

但有可能报软件定位失败错误`E:Unable to locate package nvidia-docker2` 可以通过如下命令解决

```
$ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
$ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
$ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
$ sudo apt-get update
$ sudo apt-get install -y nvidia-docker2
```

安装完毕后重启docker使之生效

```
$ sudo systemctl daemon-reload
$ sudo systemctl restart docker
```



### 1.3 将docker添加到用户组

在ubuntu中，docker默认是root用户**（即每一条docker命令都要加上sudo前缀）**

如果想要在普通用户下使用docker（**即docker命令不需要再加sudo前缀**），需要将docker添加到用户组，方法如下：

```
$ sudo gpasswd -a <your username> docker   # 将用户添加到docker组中
$ newgrp docker   # 更新docker组
$ reboot  # 重启电脑生效
```



### 1.4 vscode docker插件

VSCODE的Docker插件安装后，一般也无法立即使用，需要进行一些配置，否则直接显示`Failed to connect. Is docker running?`

原因分析：

- 原因是docker使用unix socket进行通讯，但是unix socket属于root用户，但是普通用户需要使用sudo才能开启root权限，但是普通的操作并没有root权限。

有两种配置方法：

- 方法一（**未验证过**）

  ```
  sudo groupadd docker          #添加docker用户组
  sudo gpasswd -a $USER docker  #将当前用户添加至docker用户组
  newgrp docker                 #更新docker用户组
  
  reboot                        # 重启电脑
  ```

- 方法二（**验证过**）

  ```
  sudo chmod 777 /var/run/docker.sock
  ```

  然后重启VSCODE即可生效

<img src="../imgs/37e42cfe-7620-46d2-a785-1549bb1db64c.png" alt="37e42cfe-7620-46d2-a785-1549bb1db64c" style="zoom:50%;" />

**在安装完docker后，我们同样可以在本地文件夹(apt安装的docker的默认位置为/var/lib/docker)查找到镜像images和容器containers**

==**使用`sudo docker ps -a`查询所有容器，使用`sudo docker images`查询所有镜像：如下图所示，containers下面的容器数量与查询数量一致，且id对应**==

![c5fa5a4d-7eb7-4a74-9124-fe3833bef157](../imgs/c5fa5a4d-7eb7-4a74-9124-fe3833bef157.png)






















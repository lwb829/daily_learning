# arm架构银河麒麟系统行为树依赖库安装

系统安装全程跟着如下教程：[qemu虚拟机安装银河麒麟V10 arm架构系统](https://blog.csdn.net/csdnlxc/article/details/131600433)

在飞腾平台（arm架构）下的银河麒麟系统编译运行行为树所需库已经打包，安装包地址如下：[arm架构银河麒麟系统行为树依赖库安装](https://jbox.sjtu.edu.cn/l/c1BDnj)

**以下这些依赖需要按照顺序离线解压，命令方式如下**

```
(sudo) dpkg -i ×××.deb
```

**在全部依赖解压完成后，本人验证过在behaviortree_cpp_v3目录中按照如下命令成功编译**

```
 mkdir build; cd build
 cmake ..  
 make
 sudo make install  
```



## 1. 编译工具链build-essential安装

- libstdc++-9-dev_9.3.0-10kylin2_arm64.deb
- g++-9_9.3.0-10kylin2_arm64.deb
- g++_4%3a9.3.0-11.185.1kylin2k7.5_arm64.deb
- build-essential_12.8kylin1_arm64.deb
- libmspack0_0.10.1-2_arm64.deb
- cabextract_1.9-3_arm64.deb



## 2. cmake安装

- cmake-data_3.16.3-1kylin1k3_all.deb
- libjsoncpp1_1.7.4-3.1kylin2_arm64.deb
- librhash0_1.3.9-1kylin0k1_arm64.deb
- cmake_3.16.3-1kylin1k3_arm64.deb



## 3. zmq安装

- libzmq5_4.3.2-2kylin1.20.04.1~esm2_arm64.deb
- libpgm-dev_5.2.122~dfsg-3kylin1_arm64.deb
- libsodium-dev_1.0.18-1_arm64.deb
- libnorm-dev_1.5.8+dfsg2-2build1_arm64.deb
- comerr-dev_2.1-1.45.5-2kylin2.1k0.1_arm64.deb
- libgssrpc4_1.17-6kylin5.2k0.1_arm64.deb
- libkadm5clnt-mit11_1.17-6kylin5.2k0.1_arm64.deb
- libkdb5-9_1.17-6kylin5.2k0.1_arm64.deb
- libkadm5srv-mit11_1.17-6kylin5.2k0.1_arm64.deb

- krb5-multidev_1.17-6kylin5.2k0.1_arm64.deb

- libkrb5-dev_1.17-6kylin5.2k0.1_arm64.deb
- libzmq3-dev_4.3.2-2kylin1.20.04.1~esm2_arm64.deb



## 4. boost库安装

- libboost1.71-dev_1.71.0-6kylin6k5k0.1_arm64.deb
- libboost-dev_1.71.0.0kylin2k1_arm64.deb



## 5. gtest安装

- googletest_1.10.0-2_all.deb
- libgtest-dev_1.10.0-2_arm64.deb



## 6. git安装

- git-man_1%3a2.25.1-1kylin3.11_all.deb
- liberror-perl_0.17029-1_all.deb
- git_1%3a2.25.1-1kylin3.11_arm64.deb



---

## 其它

以下这些依赖在实际行为树库依赖中并无使用，但确实存在于`/var/cache/apt/archives` 目录下，故这里也一并放上，以防之后会用上

- kylin-software-center_5.0.6.8-0k0.43_arm64.deb
- kylin-software-center-plugin-expand_5.0.6.8-0k0.43_arm64.deb
- kylin-software-center-plugin-synchrodata_5.0.6.8-0k0.43_arm64.deb
- python3-debconf_1.5.73_all.deb
- python3-debian_0.1.36kylin1_all.deb
- python3-distupgrade_1%3a20.04.18_all.deb
- python3-update-manager_1%3a20.04.9_all.deb
- qaxsafe_8.0.5-5340_arm64.deb
- ttf-mscorefonts-installer_3.7kylin6_all.deb
- ubuntu-release-upgrader-core_1%3a20.04.18_all.deb
- update-manager-core_1%3a20.04.9_all.deb
- update-notifier-common_3.192.30_all.deb
- libkimclient_1.2.2.1-0k0.6_arm64.deb




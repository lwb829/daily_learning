# 行为树可视化软件Groot依赖库安装

在联网状态下，通过如下命令进行依赖库的安装

```
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
```

在飞腾平台（arm架构）下的银河麒麟系统编译运行行为树可视化groot所需库已经打包，安装包地址如下：[行为树可视化groot依赖库](https://jbox.sjtu.edu.cn/l/u1O2ve)

这些依赖包可以通过如下命令直接安装，**会自动寻找安装**

```
sudo dpkg -i *.deb
```

共包含如下依赖包（其中少部分与行为树库依赖包中所含相同，但大多数不同，可以重复安装）

- comerr-dev_2.1-1.45.5-2kylin2.1k0.1_arm64.deb
- krb5-multidev_1.17-6kylin5.2k0.1_arm64.deb
- libdw-dev_0.176-1.1build1_arm64.deb
- libegl-dev_1.3.2-1~kylin0.20.04.2_arm64.deb
- libelf-dev_0.176-1.1build1_arm64.deb
- libgl-dev_1.3.2-1~kylin0.20.04.2_arm64.deb
- libglu1-mesa-dev_9.0.1-1build1_arm64.deb
- libglx-dev_1.3.2-1~kylin0.20.04.2_arm64.deb
- libkrb5-dev_1.17-6kylin5.2k0.1_arm64.deb
- liblzma-dev_5.2.4-1kylin2.1_arm64.deb
- libnorm-dev_1.5.8+dfsg2-2build1_arm64.deb
- libpgm-dev_5.2.122~dfsg-3kylin1_arm64.deb
- libpthread-stubs0-dev_0.4-1_arm64.deb
- libqt5concurrent5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5core5a_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5dbus5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5gui5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5network5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5opengl5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5opengl5-dev_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5printsupport5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5sql5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5svg5_5.12.12-0kylin1k0.1_arm64.deb
- libqt5svg5-dev_5.12.12-0kylin1k0.1_arm64.deb
- libqt5test5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5widgets5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libqt5xml5_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- libsodium-dev_1.0.18-1_arm64.deb
- libvulkan-dev_1.2.131.2-1_arm64.deb
- libx11-6_2%3a1.6.9-2kylin1.5_arm64.deb
- libx11-dev_2%3a1.6.9-2kylin1.5_arm64.deb
- libxau-dev_1%3a1.0.9-0kylin1_arm64.deb
- libxcb1-dev_1.14-2_arm64.deb
- libxdmcp-dev_1%3a1.1.3-0kylin1_arm64.deb
- libxext-dev_2%3a1.3.4-0kylin1_arm64.deb
- libzmq3-dev_4.3.2-2kylin1.20.04.1~esm2_arm64.deb
- libzmq5_4.3.2-2kylin1.20.04.1~esm2_arm64.deb
- qt5-qmake_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- qt5-qmake-bin_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- qtbase5-dev_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- qtbase5-dev-tools_5.12.12+dfsg-0kylin1k0.5_arm64.deb
- qtchooser_66-2build1_arm64.deb
- x11proto-core-dev_2019.2-1kylin1_all.deb
- x11proto-dev_2019.2-1kylin1_all.deb
- x11proto-xext-dev_2019.2-1kylin1_all.deb
- xorg-sgml-doctools_1%3a1.11-1_all.deb
- xtrans-dev_1.4.0-1_all.deb
- zlib1g-dev_1%3a1.2.11.dfsg-2kylin1.5k0.2_arm64.deb


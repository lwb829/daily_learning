# autoware.ai工作目录

- **build**：编译过程生成的中间文件
- **install**：编译过程生成，可执行文件、配置等，部署时移植该文件夹
- **log**：编译过程生成日志，debug时用到
- **src**：最为核心，包括各种代码，启动文件等
  - **autoware**：主要需要开发的内容
    - **common**：比较通用的功能
    - **core_perception**：感知、定位相关
    - **core_planning**：规划、控制相关
    - **documentation**：文档，如演示demo文档
    - **messages**：接口、自定义msg数据结构
    - **simulation**：仿真
    - **utilities**：类似于common，比较通用的工具
    - **visualization**：可视化
  - **car_demo**：仿真相关
  - **citysim**：仿真相关，车、世界、场景
  - **drivers**：与上车实际测试相关
  - **vendor**：与上车实际测试相关

**注意：关于launch文件的几点**

- 在autoware项目里，修改一个launch文件，也需要重新对这个包编译
- launch中给予的参数，优先级是最高的
- 不在终端打印，就加output='screen'



# 利用colcon编译autoware工作空间

- 不带CUDA

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 带CUDA

```
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 特殊后缀

  - 选择某个包单独编译，例如

  ```
  (AUTOWARE_COMPILE_WITH_CUDA=1) colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select lidar_localizer
  ```

  - 忽略某个包编译，例如

  ```
  (AUTOWARE_COMPILE_WITH_CUDA=1) colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore lanelet2_python
  ```



# 官方demo运行

- 准备工作

1. 将bag文件存放在autoware目录下
2. 将data文件放在home路径下的.autoware文件夹中（其中包括一些点云地图，矢量地图的原始信息等）

- 开始运行

**注意：每个模块开启前都要配置环境变量**

```
source install/setup.bash
```

1. 地图

   ```
   roslaunch autoware_quickstart_examples ××_map.launch
   ```

2. 定位

   ```
   roslaunch autoware_quickstart_examples ××_localization.launch
   ```

3. 可视化

   ```
   rviz
   ```

   若不出现，则在File中手动加载rviz配置文件

4. 跑包

   ```
   #进入到bag文件目录下
   rosbag play sample_moriyama_150324.bag
   ```

5. 感知

   ```
   roslaunch autoware_quickstart_examples ××_detection.launch
   ```

6. 规划

   ```
   roslaunch autoware_quickstart_examples ××_mission_planning.launch
   ```

7. 运动

   ```
   roslaunch autoware_quickstart_examples ××_motion_planning.launch
   ```

   


# 利用colcon编译autoware工作空间

- 不带CUDA

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 带CUDA

```
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

-   特殊后缀

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

   


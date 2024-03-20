# 在Autoware中使用带GPS的仿真模型

> 现对autoware.1.13的仿真包中有包含GPS的车辆模型，但需要做一些修改，详细使用如下。



## 1.在车辆模型中修改GPS配准参数

带GPS的车辆模型 `vehicle_with_gps.xacro` 位于`autoware/visualization/vehicle_model/urdf`

其原本1.13的配准文件为`calibration.yaml`：

```
<!-- ================ sensor ================ -->
<xacro:property name="calibration" value="${load_yaml('$(find vehicle_model)/config/calibration.yaml')}"/>
```

其中不包括GPS的配准信息，需要载入包含其信息的`calibration_with_gps.yaml`：

```yaml
base_link2camera:
  x: 2.0
  y: 0.0
  z: 1.35
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
base_link2velodyne:
  x: 1.2
  y: 0.0
  z: 2.0
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
base_link2imu:
  x: 1.2
  y: 0.0
  z: 1.8
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
#新增GPS
base_link2gps:
  x: 0.0
  y: 0.0
  z: 2.0
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

故修改车辆模型`vehicle_with_gps.xacro`为：

```
<!-- ================ sensor ================ -->
<xacro:property name="calibration" value="${load_yaml('$(find vehicle_model)/config/calibration_with_gps.yaml')}"/>
```

并在仿真环境的launch文件中添加进车辆模型，路径为

`autoware/simulation/vehicle_gazebo_simulation_launcher/launch`

```
...
<arg name="model" default="$(find vehicle_model)/urdf/vehicle_with_gps.xacro"/>
<arg name="ns" default="/autoware_gazebo"/>
...
```



## 2.启动仿真环境，查看GPS输出话题中的消息

在 `vehicle_with_gps.xacro` 中定义了GPS输出的话题为 `/fix` ，所以直接启动修改后的仿真环境查看话题输出：

```
$ roslaunch vehicle_gazebo_simulation_launcher world_test.launch 
```

```
$ rostopic echo /fix
header: 
  seq: 4618
  stamp: 
    secs: 4581
    nsecs: 200000000
  frame_id: "gps"
status: 
  status: 0
  service: 0
latitude: 49.9000001238
longitude: 8.89999999999
altitude: 0.0337405377425
position_covariance: [9.999999999999998e-15, 0.0, 0.0, 0.0, 9.999999999999998e-15, 0.0, 0.0, 0.0, 9.999999999999998e-15]
position_covariance_type: 2
```

从中我们可以看到经纬度消息



## 3. 修改仿真环境的原点经纬度，以及程序参考点的修改

需要将经纬度信息解析成XYZ坐标，则需要参考点作为解析的原点。在autoware中设置了19个参考点，一般来说参考点都尽量选择到离实验环境近一些的位置，这样解析出来的XYZ坐标值就不会特别大。



### 3.1 修改仿真环境的原点经纬度

相当于指定这个仿真环境的经纬度信息，目前使用的是 `openPlanner.world`，路径为：`autoware/visualization/gazebo_world_description/worlds/openPlanner.world`

修改部分从第96行开始

```
...
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <latitude_deg>49.9</latitude_deg>
  <longitude_deg>8.9</longitude_deg>
  <elevation>0</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
...
```



### 3.2在解析文件中添加参考点

参考点在 `gnss` 功能包的 `geo_pos_conv.cpp` 中进行设置，具体路径为：

`autoware/common/gnss/src/geo_pos_conv.cpp`

添加部分为：

```
...
  else if(num == 20)
  {
    lon_deg = 49.9;
    lon_min = 0;
    lat_deg = 8.9;
    lat_min = 0;
  }
...
```



### 3.3 在具体的定位程序中使用添加的参考点

定位功能包为 `gnss_localizer` ，仿真环境依赖其下节点 `fix2tfpose`。所以修改 `fix2tfpose.launch` 文件为：

```
<launch>
  <arg name="plane" default="20"/> 
  <!--使用第20号参考点-->
  <node pkg="gnss_localizer" type="fix2tfpose" name="fix2tfpose" output="log">
    <param name="plane" value="$(arg plane)"/>
  </node>
</launch>
```



## 4. 查看仿真环境定位信息

编译以上涉及到的功能包，启动仿真环境，从终端启动 `fix2tfpose.launch`。从话题 `/gnss_pose` 查看定位信息：

```
linkx@linkx:~/autoware.1.13$ rostopic echo /gnss_pose
header: 
  seq: 28
  stamp: 
    secs: 5977
    nsecs: 200000000
  frame_id: "map"
pose: 
  position: 
    x: 0.0274116436603
    y: 10.4625784624
    z: 0.0337177645354
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.706179872464
    w: 0.708032476463
```
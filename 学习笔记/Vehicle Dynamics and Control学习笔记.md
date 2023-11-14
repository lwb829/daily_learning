# Vehicle Dynamics and Control

## Chapter 1  Introduction

### 1.2  ACTIVE STABILITY CONTROL SYSTEMS（主动稳定控制系统）

yaw stability control 偏航稳定性控制

roll over stability control 侧翻稳定性控制

**对于 Figure1-1中的摩擦系数u对比**：u越高，越能满足车辆过弯所需的侧向力，则越容易满足驾驶员的转向输入所遵循的轨迹；u越低，车辆越容易沿着半径更大的轨迹拐弯，特别地，当u特别低时，它不可能完全实现标称偏航率运动。



### 1.4  TECHNOLOGIES FOR ADDRESSING TRAFFIC CONGESTION

#### 1.4.2 “Traffic-friendly” adaptive cruise control

- **ACC**：主要利用了雷达技术，通过毫米波雷达，发射毫米波段的电磁波，利用障碍物反射波的时间差确定障碍物距离，利用反射波的频率偏移确定相对速度。

- **原理：距离测量—>确定前车速度—>确定前车位置—>确定调节车辆**



#### 1.4.3 Narrow tilt-controlled commuter vehicles

**ITS**：intelligent transportation systems 智慧交通系统



### 1.5  EMISSIONS AND FUEL ECONOMY

#### 1.5.1 Hybrid electric vehicles（HEV）混合动力电动汽车

- 优点：结合传统**内燃机**（ICE）和**电动机**，可以获得更大的续航里程，并减轻排放量，提高燃油经济性。

- 动力系统：
  - 串联：汽油**发动机带动发电机**，发电机可以为电池充电，也可以为驱动变速器的电动机提供动力
  - 并联：燃气发动机和电动机都独立连接到变速器上，二者均可提供推进动力



## Chapter 2  Lateral Vehicle Dynamics 车辆横向动力学

### 2.1 LATERAL SYSTEMS UNDER COMMERCIAL DEVELOPMENT

- 解决车道偏离事故的三种类型的横向系统：

  - 车道偏离预警系统（LDWS）

    利用AutoVue设备，识别道路和车道标记之间的区别，计算机将这些数据与车辆速度结合，预测车辆何时开始漂移到意外的变道，并提醒驾驶员进行纠正。

  - 车道保持系统（LKS）

    自动控制方向盘，使车辆保持在车道内，并在转弯时跟随车道行驶。

  - 偏航稳定控制系统 （YSCS）

    三种对于偏航控制的稳定控制系统：

    - Differential Braking Systems（差速制动系统）：利用车辆上的ABS制动系统，在左右轮之间施加差速制动来控制偏航力矩
    - Steer-by-Wire Systems（线控转向系统）：修改驾驶员的转向角度输入，并为车轮增加一个校正转向角度
    - Active Torque Distribution Systems（主动扭矩分配系统）：利用主动差速器和全轮驱动技术来独立控制分配给每个车轮的驱动扭矩，从而提供牵引力和偏航力矩的主动控制
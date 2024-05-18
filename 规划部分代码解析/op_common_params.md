# 功能

主要关注launch文件中的各个参数及其表示含义，这里选择其中一些比较重要的参数（其余参数为多与车辆自身相关）

- `Plan Distance` -> `maxLocalPlanDistance` 用于设置从全局轨迹两侧的衍生出的局部轨迹的长度

- `Path Density` -> `pathDensity` 用于局部轨迹上两个轨迹点的距离
- `Horizontal Density` -> `rollOutDensity` 用于设置两个局部轨迹的间距
- `Rollouts Number` -> `rollOutsNumber` 用于设置局部轨迹的数量
- `Max Velocity` -> `maxVelocity` 轨迹上的执行速度，避障时局部轨迹速度减半，单位m/s
- `Acceleration` -> `maxAcceleration` 在轨迹上行驶时的最大加速度
- `Deceleration` -> `maxDeceleration` 在轨迹上行驶时的最大减速度
- `Enable Following` -> `enableFollowing` 只是跟随模式，即只进行全局路径的跟踪
- `Enable Avoidance` -> `enableSwerving` 启用避障模式，即为 True 的话就可以产生 Rollouts，否则就只看 enableFollowing 的情况
- **`Follow Distance` -> `minFollowingDistance` 关键参数，沿着轨迹设置从多远检测障碍物**
- **`Avoiding Distance` -> `minDistanceToAvoid` 感知到路径上的障碍物，判断多远开始绕行**
- **`Avoidance Limit` -> `maxDistanceToAvoid` 障碍物距离多远可以开车**
- **`Lateral Safety` -> `horizontalSafetyDistance` “侧向安全距离”，认为是车辆的安全框的宽度**
- **`Longitudial Safety` -> `verticalSafetyDistance` “纵向安全距离”，可以认为认为是车辆的安全框的长度**






































































































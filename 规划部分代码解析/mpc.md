# mpc节点

## 车辆模型

### `vehicle_model_interface.cpp`

路径为：`autoware-1.14\src\autoware\core_planning\mpc_follower\src\vehicle_model\vehicle_model_interface.cpp`

定义了一个名为 `VehicleModelInterface` 的类

**是一个为车辆建模的接口类，提供了一组用于车辆动力学建模和控制的基础设施和虚函数**

提供了基础功能的实现，包括构造函数、状态向量 `x`、输入向量 `u` 和输出向量 `y` 的维度获取，以及设置车辆速度和路径曲率的方法。

比较关键的计算离散矩阵的公式部分如下

```c++
  /**
   * @brief calculate discrete model matrix of x_k+1 = Ad * xk + Bd * uk + Wd, yk = Cd * xk 
   * @param [out] Ad coefficient matrix
   * @param [out] Bd coefficient matrix
   * @param [out] Cd coefficient matrix
   * @param [out] Wd coefficient matrix
   * @param [in] dt Discretization time
   */
  virtual void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, const double &dt) = 0;
```

这是一个纯虚函数，用于描述车辆的运动模型，**形式为离散时间状态空间模型 ==`x_k+1 = Ad * xk + Bd * uk + Wd`，`yk = Cd * xk`==**。**`Ad`、`Bd`、`Cd` 和 `Wd` 分别代表状态转移矩阵、控制输入矩阵、输出矩阵和干扰矩阵**。参数 `dt` 表示离散化时间步长



### `vehicle_model_bicycle_kinematics.cpp`

#### 1.构造函数

```c++
KinematicsBicycleModel::KinematicsBicycleModel(const double &wheelbase, const double &steer_lim, const double &steer_tau)
    : VehicleModelInterface(/* dim_x */ 3, /* dim_u */ 1, /* dim_y */ 2)
{
    wheelbase_ = wheelbase;
    steer_lim_ = steer_lim;
    steer_tau_ = steer_tau;
};
```

通过继承 `VehicleModelInterface` 类，初始化了车辆运动学模型的基本参数

- 参数值设置
  - `wheelbase`：车辆的轴距
  - `steer_lim`：转向角度的最大限制
  - `steer_tau`：转向动作的时间常数

- 参数维度设置
  - `dim_x` = 3：状态向量 `x` 的维度。对于基本的自行车模型，这可能代表了车辆的位置和朝向，例如 `[x, y, theta]`。
  - `dim_u` = 1：输入向量 `u` 的维度。这通常代表了单一的控制输入，比如转向角。
  - `dim_y` = 2：输出向量 `y` 的维度。对于许多车辆模型，输出可能包括车辆的位置信息 `[x, y]`。



#### 2.函数`calculateDiscreteMatrix`

```c++
void KinematicsBicycleModel::calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, const double &dt)
{
    auto sign = [](double x) { return (x > 0.0) - (x < 0.0); };

    /* Linearize delta around delta_r (referece delta) */
    double delta_r = atan(wheelbase_ * curvature_);
    if (abs(delta_r) >= steer_lim_)
        delta_r = steer_lim_ * (double)sign(delta_r);
    double cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));

    Ad << 0.0, velocity_, 0.0,
        0.0, 0.0, velocity_ / wheelbase_ * cos_delta_r_squared_inv,
        0.0, 0.0, -1.0 / steer_tau_;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    Ad = (I - dt * 0.5 * Ad).inverse() * (I + dt * 0.5 * Ad); // bilinear discretization

    Bd << 0.0, 0.0, 1.0 / steer_tau_;
    Bd *= dt;

    Cd << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0;

    Wd << 0.0,
        -velocity_ * curvature_ + velocity_ / wheelbase_ * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
        0.0;
    Wd *= dt;
}
```

采用了线性化和离散化技术来适应车辆的运动学模型，计算离散时间运动学模型的系数矩阵

- 定义了一个名为`sign`的**lambda表达式**，用来计算一个双精度浮点数`x`的符号。如果`x`大于0，返回1；如果`x`小于0，返回-1；如果`x`等于0，返回0

- 检查计算出的转向角度`delta_r`是否超过了某个预设的转向限制`steer_lim_`。如果是，就将`delta_r`设置为`steer_lim_`乘以`delta_r`的符号，这样可以保证`delta_r`的绝对值不超过限制值，同时保留其方向（正负符号）

  **由这里的`delta_r`公式可以看出，此处所选择的车辆运动学模型为：以车辆后轴为中心**

- 使用双线性变换（Tustin方法）对连续时间模型进行离散化。这种方法通过替换微分操作为差分操作来适应数字实现，具体表现为 `(I - dt * 0.5 * Ad).inverse() * (I + dt * 0.5 * Ad)`



#### 3.函数`calculateReferenceInput`

```c++
void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd &Uref)
{
    Uref(0, 0) = std::atan(wheelbase_ * curvature_);
}
```

计算并设置参考输入 `Uref`，被用作目标或期望的控制输入值

- 通过 `atan(wheelbase_ * curvature_)` 实现，计算结果是车辆为了沿当前路径行驶所需的理想转向角

- 将计算得到的理想转向角度赋值给 `Uref` 矩阵的第一个元素 `Uref(0, 0)`

  **这表示在该模型中，控制输入仅包括一个量，即转向角**



### `vehicle_model_bicycle_dynamics.cpp`

#### 1.构造函数

```c++
DynamicsBicycleModel::DynamicsBicycleModel(double &wheelbase, double &mass_fl, double &mass_fr, double &mass_rl, double &mass_rr, double &cf, double &cr): VehicleModelInterface(/* dim_x */ 4, /* dim_u */ 1, /* dim_y */ 2)
{
    wheelbase_ = wheelbase;

    const double mass_front = mass_fl + mass_fr;
    const double mass_rear = mass_rl + mass_rr;

    mass_ = mass_front + mass_rear;
    lf_ = wheelbase_ * (1.0 - mass_front / mass_);
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
    cf_ = cf;
    cr_ = cr;
};
```

- 关键参数：
  - `cf`, `cr` ：分别表示前轮和后轮的侧向刚度
  - `lf_` 和 `lr_` ：分别表示质心到前轴和后轴的距离
  - `iz_` ：计算了车辆绕其质心的z轴（垂直于地面的轴）的转动惯量



#### 2.函数`calculateDiscreteMatrix`

```c++
void DynamicsBicycleModel::calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd, Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, const double &dt)
{
    /*
     * x[k+1] = Ad*x[k] + Bd*u + Wd
     */

    const double vel = std::max(velocity_, 0.01);

    Ad = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
    Ad(0, 1) = 1.0;
    Ad(1, 1) = -(cf_ + cr_) / (mass_ * vel);
    Ad(1, 2) = (cf_ + cr_) / mass_;
    Ad(1, 3) = (lr_ * cr_ - lf_ * cf_) / (mass_ * vel);
    Ad(2, 3) = 1.0;
    Ad(3, 1) = (lr_ * cr_ - lf_ * cf_) / (iz_ * vel);
    Ad(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    Ad(3, 3) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (iz_ * vel);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
    Eigen::MatrixXd Ad_inverse = (I - dt * 0.5 * Ad).inverse();

    Ad = Ad_inverse * (I + dt * 0.5 * Ad); // bilinear discretization

    Bd = Eigen::MatrixXd::Zero(dim_x_, dim_u_);
    Bd(0, 0) = 0.0;
    Bd(1, 0) = cf_ / mass_;
    Bd(2, 0) = 0.0;
    Bd(3, 0) = lf_ * cf_ / iz_;

    Wd = Eigen::MatrixXd::Zero(dim_x_, 1);
    Wd(0, 0) = 0.0;
    Wd(1, 0) = (lr_ * cr_ - lf_ * cf_) / (mass_ * vel) - vel;
    Wd(2, 0) = 0.0;
    Wd(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (iz_ * vel);

    Bd = (Ad_inverse * dt) * Bd;
    Wd = (Ad_inverse * dt * curvature_ * vel) * Wd;

    Cd = Eigen::MatrixXd::Zero(dim_y_, dim_x_);
    Cd(0, 0) = 1.0;
    Cd(1, 2) = 1.0;
}
```

- **速度处理**：使用 `std::max(velocity_, 0.01)` 确保计算中的速度值不为零，避免除以零的错误。
- 同样使用双线性变化法，将连续模型变换到离散模型
- 通过与 Ad 相同的转换方法更新 Bd 和 Wd



#### 3.函数`calculateReferenceInput`

```c++
void DynamicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd &Uref)
{
    const double vel = std::max(velocity_, 0.01);
    const double Kv = lr_ * mass_ / (2 * cf_ * wheelbase_) - lf_ * mass_ / (2 * cr_ * wheelbase_);
    Uref(0, 0) = wheelbase_ * curvature_ + Kv * vel * vel * curvature_;
}

```

- 计算系数 `Kv`，该系数考虑了车辆的物理参数，如车辆质量 `mass_`，前轮和后轮的侧偏刚度 `cf_` 和 `cr_`，以及车轮基地（轴距）`wheelbase_`。`Kv` 反映了车速变化和路径曲率对转向需求的影响。

- `Uref(0, 0)` 被设置为一个与车辆轴距、路径曲率和速度平方成比例的值。这个表达式考虑了车速较高时，为了保持车辆沿着给定的曲率行驶，可能需要更大的转向角调整。



## mpc_trajectory

**强调了mpc控制轨迹需要由以下几个同样大小维度的参数：**

- **`x`**: 存储车辆在三维空间中的 x 坐标位置序列。
- **`y`**: 存储车辆在三维空间中的 y 坐标位置序列。
- **`z`**: 存储车辆在三维空间中的 z 坐标位置序列。
- **`yaw`**: 存储车辆的偏航角（航向角）序列。
- **`vx`**: 存储车辆在x方向（或前进方向）上的速度序列。
- **`k`**: 存储车辆路径的曲率序列。
- **`relative_time`**: 存储从起点开始的相对时间序列。



## mpc控制

![image-20240216144517920](../学习笔记/imgs/image-20240216144517920.png)



### 1.`mpc_waypoints_converter.cpp`

路径为：`autoware-1.14\src\autoware\core_planning\mpc_follower\src\mpc_waypoints_converter.cpp`

- 订阅话题（输入）

  ```c++
  sub_closest_waypoint_ = nh_.subscribe("/closest_waypoint", 1, &MPCWaypointsConverter::callbackClosestWaypoints, this);
  sub_base_waypoints_ = nh_.subscribe("/base_waypoints", 1, &MPCWaypointsConverter::callbackBaseWaypoints, this);
  sub_final_waypoints_ = nh_.subscribe("/final_waypoints", 1, &MPCWaypointsConverter::callbackFinalWaypoints, this);
  ```

  用来发布局部路径信息

- 发布话题（输出）

  ```c++
  pub_waypoints_ = nh_.advertise<autoware_msgs::Lane>("/mpc_waypoints", 1);
  pub_reverse_ = nh_.advertise<std_msgs::Bool>("/mpc_reverse", 1);
  ```

  

#### 回调函数`callbackFinalWaypoints`

```c++
void callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints)
  {
    if (base_waypoints_.waypoints.size() == 0 || final_waypoints.waypoints.size() == 0)
      return;

    if ((int)base_waypoints_.waypoints.size() - 1 < closest_idx_)
    {
      ROS_WARN("base_waypoints_.waypoints.size() - 1 = %d, closest_idx_ = %d", (int)base_waypoints_.waypoints.size(), closest_idx_);
      return;
    }

    auto sq_dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return dx * dx + dy * dy;
    };
    int count_negetive = 0;
    for(auto &p:final_waypoints.waypoints)
    {
        if(p.twist.twist.linear.x<0)
        {
          count_negetive++;
        }
    }
    if(float(count_negetive)/float(final_waypoints.waypoints.size())>0.95)
    {
      is_reverse_ = true;
    }

    autoware_msgs::Lane mpc_waypoints;
    mpc_waypoints.header = final_waypoints.header;
    mpc_waypoints.increment = final_waypoints.increment;
    mpc_waypoints.lane_id = final_waypoints.lane_id;
    mpc_waypoints.lane_index = final_waypoints.lane_index;
    mpc_waypoints.cost = final_waypoints.cost;
    mpc_waypoints.closest_object_distance = final_waypoints.closest_object_distance;
    mpc_waypoints.closest_object_velocity = final_waypoints.closest_object_velocity;
    mpc_waypoints.is_blocked = final_waypoints.is_blocked;

    // find closest point index in base_waypoints (topic /closest_waypoints has no consistency with /final_waypoints due to delay)
    int closest_idx = -1;
    for (int i = 0; i < (int)base_waypoints_.waypoints.size(); ++i) {
      const double d = sq_dist(final_waypoints.waypoints[1].pose.pose.position, base_waypoints_.waypoints[i].pose.pose.position);
      if (d < 0.01) {
        closest_idx = i;
        break;
      }
    }
    if (closest_idx == -1) {
      ROS_ERROR("cannot find closest base_waypoints' waypoint to final_waypoints.waypoint[1] !!");
    }

    int base_start = std::max(closest_idx - back_waypoints_num_, 0);
    for (int i = base_start; i < closest_idx; ++i)
    {
      mpc_waypoints.waypoints.push_back(base_waypoints_.waypoints.at(i));
      mpc_waypoints.waypoints.back().twist = final_waypoints.waypoints[1].twist;
    }

    int final_end = std::min(front_waypoints_num_ + 1, (int)final_waypoints.waypoints.size());
    for (int i = 1; i < final_end; ++i)
    {
      mpc_waypoints.waypoints.push_back(final_waypoints.waypoints.at(i));
    }


    std_msgs::Bool reverse_msg;
    reverse_msg.data = is_reverse_;
    pub_reverse_.publish(reverse_msg);
    pub_waypoints_.publish(mpc_waypoints);
  }
```

- **检查路径点有效性**：函数开始时先检查 `base_waypoints_` 和 `final_waypoints` 是否为空，如果是，则直接返回不进行处理。
- **计算负速度比例**：遍历 `final_waypoints`，统计速度小于0（即倒车速度）的路径点数量。**如果这些倒车路径点的比例超过95%，则设置 `is_reverse_` 为 `true`，表示车辆应该进行倒车**。

- **构建MPC路径点**：复制 `final_waypoints` 的头信息和其他元数据到新的 `mpc_waypoints` 消息中。
- **重算`closest_waypoints`**：这是由于发布轨迹并不是连续实时的，接收时closet_waypoints可能已经改变。

- **合并路径点**：添加 `final_waypoints` 中前面一定数量的路径点到 `mpc_waypoints`。

mpc_waypoints将被用于真正mpc控制的参考轨迹，同时还需要输入车辆当前的位姿与速度



### 2.`mpc_follower_core.cpp`

111


























































































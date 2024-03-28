# pure_pursuit节点

## pure_pursuit.launch

```xml
  <arg name="is_linear_interpolation" default="True"/>
  <arg name="publishes_for_steering_robot" default="True"/>
  <arg name="add_virtual_end_waypoints" default="True"/>
  <arg name="const_lookahead_distance" default="4.0"/>
  <arg name="const_velocity" default="5.0"/>
  <arg name="lookahead_ratio" default="2.0"/>
  <arg name="minimum_lookahead_distance" default="6.0"/>
  <arg name="minimum_reverse_lookahead_distance" default="3.0"/>
  <!-- 0 = waypoints, 1 = provided constant velocity -->
  <arg name="velocity_source" default="0"/>
```

上述均为参数(args)

- **is_linear_interpolation**：是否使用线性插值来计算路径点之间的目标点。默认值为`True`。
- **publishes_for_steering_robot**：是否发布用于转向控制的信息。默认值为`True`。
- **add_virtual_end_waypoints**：是否在路径末尾添加虚拟的终点，以帮助路径跟踪。默认值为`True`。
- **const_lookahead_distance**：设置一个恒定的前瞻距离，单位为米。默认值为`4.0`米。
- **const_velocity**：设置一个恒定的速度，单位为米/秒。默认值为`5.0`米/秒。
- **lookahead_ratio**：设置前瞻距离的比例系数，用于动态调整前瞻距离。默认值为`2.0`。
- **minimum_lookahead_distance**：设置最小前瞻距离，单位为米。默认值为`6.0`米。
- **minimum_reverse_lookahead_distance**：设置倒车时的最小前瞻距离，单位为米。默认值为`3.0`米。
- **velocity_source**：速度来源的选择，`0`代表从路径点中提取，`1`代表使用提供的恒定速度。默认值为`0`。

```xml
  <node pkg="pure_pursuit" type="pure_pursuit" name="pure_pursuit" output="screen">
    <param name="is_linear_interpolation" value="$(arg is_linear_interpolation)"/>
    <param name="publishes_for_steering_robot" value="$(arg publishes_for_steering_robot)"/>
    <param name="add_virtual_end_waypoints" value="$(arg add_virtual_end_waypoints)"/>
    <param name="const_lookahead_distance" value="$(arg const_lookahead_distance)"/>
    <param name="const_velocity" value="$(arg const_velocity)"/>
    <param name="lookahead_ratio" value="$(arg lookahead_ratio)"/>
    <param name="minimum_lookahead_distance" value="$(arg minimum_lookahead_distance)"/>
    <param name="minimum_reverse_lookahead_distance" value="$(arg minimum_reverse_lookahead_distance)"/>
    <param name="velocity_source" value="$(arg velocity_source)"/>
  </node>
```

一系列的`<param>`标签，它们通过`$(arg arg_name)`语法获取上面定义的参数值，并将这些值设置为节点运行时的参数。这些参数控制着纯追踪算法的各个方面，比如是否进行线性插值、是否发布转向控制信息、前瞻距离的大小等等。



## pure_pursuit.cpp

路径为`autoware-1.14\src\autoware\core_planning\pure_pursuit\src\pure_pursuit.cpp`



### 1.PurePursuit构造函数

```c++
PurePursuit::PurePursuit()
  : RADIUS_MAX_(9e10)
  , KAPPA_MIN_(1 / RADIUS_MAX_)
  , is_linear_interpolation_(false)
  , next_waypoint_number_(-1)
  , lookahead_distance_(0)
  , minimum_lookahead_distance_(6)
  , current_linear_velocity_(0)
{
}
```

在C++中用于初始化类的一个实例。构造函数在创建类的对象时自动调用，用于设置成员变量的初始值。

**这是C++中推荐的初始化方式，因为它比在构造函数体内赋值更高效。**

- **RADIUS_MAX_(9e10)**：设置一个非常大的最大半径值，`9e10`是科学记数法，表示9×10^10。

- **KAPPA_MIN(1 / RADIUS_MAX)**：根据最大半径计算最小曲率值。由于最大半径非常大，这里的最小曲率将非常接近0，但不为0，用于确保在几乎直行的情况下有一个非零的曲率值。

- **next_waypoint_number_(-1)**：初始化下一个路径点的编号为-1，这通常表示尚未设置或找到下一个路径点。
- **is_linear_interpolation_(false)**：初始化线性插值标志为`false`。这意味着，默认情况下不使用线性插值来计算路径点之间的目标点。

- **lookahead_distance_(0)**：初始化前瞻距离为0。这个值在运行过程中会被更新，以动态调整车辆的前瞻距离。
- **minimum_lookahead_distance_(6)**：设置最小前瞻距离为6米。这是为了确保车辆至少关注位于它前方6米处的路径点，以保持平稳和安全的行驶。
- **current_linear_velocity_(0)**：初始化当前线性速度为0，表示车辆起始时静止。



### 2.函数`calcCurvature`

```c++
double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  double kappa;
  double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;

  if (denominator != 0)
  {
    kappa = numerator / denominator;
  }
  else
  {
    if (numerator > 0)
    {
      kappa = KAPPA_MIN_;
    }
    else
    {
      kappa = -KAPPA_MIN_;
    }
  }
  // ROS_INFO("kappa : %lf", kappa);
  return kappa;
}
```

计算从当前位置到目标点的曲率（κ）

- 需要参数`geometry_msgs::Point target`：代表三维空间中的一个点，即目标点

- **计算分母**：使用`getPlaneDistance`函数计算当前位置（`current_pose_.position`）到目标点（`target`）的平面距离的平方
- **计算分子**：通过`calcRelativeCoordinate`函数计算目标点相对于当前位置的坐标，然后取这个相对坐标的y分量乘以2作为分子。这一步是为了计算出目标点在车辆坐标系中的横向偏移量。
- **计算曲率（κ）**：
  - 如果分母不为0，曲率κ等于分子除以分母。
  - 如果分母为0（意味着目标点正好在车辆正前方，没有横向偏移），则曲率κ的值取决于分子的符号。如果分子大于0，曲率设为一个预定义的最小曲率值`KAPPA_MIN_`。如果分子小于或等于0，则曲率为`-KAPPA_MIN_`



### 3.函数`interpolateNextTarget`

```c++
bool PurePursuit::interpolateNextTarget(
  int next_waypoint, geometry_msgs::Point* next_target) const
{
  constexpr double ERROR = pow(10, -5);  // 0.00001

  int path_size = static_cast<int>(current_waypoints_.size());
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.at(next_waypoint).pose.pose.position;
    return true;
  }
  double search_radius = lookahead_distance_;
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end =
    current_waypoints_.at(next_waypoint).pose.pose.position;
  geometry_msgs::Point start =
    current_waypoints_.at(next_waypoint - 1).pose.pose.position;

  // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2),
  // a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
  if (!get_linear_flag)
    return false;

  // let the center of circle be "(x0,y0)", in my code ,
  // the center of circle is vehicle position
  // the distance  "d" between the foot of
  // a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);

  // ROS_INFO("a : %lf ", a);
  // ROS_INFO("b : %lf ", b);
  // ROS_INFO("c : %lf ", c);
  // ROS_INFO("distance : %lf ", d);

  if (d > search_radius)
    return false;

  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v
  // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);
  // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);

  // the foot of a perpendicular line
  geometry_msgs::Point h1;
  h1.x = current_pose_.position.x + d * unit_w1.getX();
  h1.y = current_pose_.position.y + d * unit_w1.getY();
  h1.z = current_pose_.position.z;

  geometry_msgs::Point h2;
  h2.x = current_pose_.position.x + d * unit_w2.getX();
  h2.y = current_pose_.position.y + d * unit_w2.getY();
  h2.z = current_pose_.position.z;

  // ROS_INFO("error : %lf", error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
  // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

  // check which of two foot of a perpendicular line is on the line equation
  geometry_msgs::Point h;
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }

  // get intersection[s]
  // if there is a intersection
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    target1.z = current_pose_.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = current_pose_.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    double interval = getPlaneDistance(end, start);
    if (getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
}
```

用于计算路径上的下一个目标点。它通过线性插值和几何运算来找到与车辆当前位置相关的最合适的路径点。

- **初始化和边界条件检查**：函数开始时，定义了一个极小值`ERROR`用于后续的浮点数比较。接着，检查`next_waypoint`是否为路径的最后一个点，如果是，则直接使用这个点作为下一个目标点。
- **计算直线与以车辆为中心的圆的交点**：
  - 首先，根据车辆位置到直线的垂直距离（`d`），判断这个距离是否大于前瞻距离`search_radius`。如果大于，意味着路径点不在车辆的前瞻范围内，函数返回`false`。
  - 使用向量运算找到垂足点，即从车辆位置到路径直线最近的点。
  - 如果垂足点的距离正好等于前瞻距离，那么垂足点就是目标点。
  - 如果不是，需要进一步计算两个交点（圆与直线的交点），并确定哪个交点在车辆前方。

- **选择正确的交点**：通过比较两个交点与路径终点的距离，来确定哪个交点是合适的目标点。选择距离较小的那个点，因为它更接近于车辆当前的前方。



### 4.函数getNextWaypoint

```c++
void PurePursuit::getNextWaypoint()
{
  int path_size = static_cast<int>(current_waypoints_.size());

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    next_waypoint_number_ = -1;
    return;
  }

  // look for the next waypoint.
  for (int i = 0; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      next_waypoint_number_ = i;
      return;
    }

    // if there exists an effective waypoint
    if (getPlaneDistance(
      current_waypoints_.at(i).pose.pose.position, current_pose_.position)
      > lookahead_distance_)
    {
      next_waypoint_number_ = i;
      return;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  next_waypoint_number_ = -1;
  return;
}
```

功能是获取下一个路径点的索引

- `if (path_size == 0)`：如果路径点列表为空，则将 `next_waypoint_number_` 设置为 -1，表示未找到下一个路径点，然后退出函数。
- 接下来通过循环遍历路径点列表 `current_waypoints_`，对每个路径点执行以下操作：
  - `getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position)`：计算当前车辆位置 `current_pose_` 和路径点位置 `current_waypoints_.at(i).pose.pose.position` 之间的平面距离。
  - 检查平面距离是否大于预设的前瞻距离 `lookahead_distance_`。
  - 如果找到第一个平面距离大于前瞻距离的路径点，则将 `next_waypoint_number_` 设置为当前路径点的索引，并退出循环。



### 5.函数`canGetCurvature`

```c++
bool PurePursuit::canGetCurvature(double* output_kappa)
{
  // search next waypoint
  getNextWaypoint();
  if (next_waypoint_number_ == -1)
  {
    ROS_INFO("lost next waypoint");
    return false;
  }
  // check whether curvature is valid or not
  bool is_valid_curve = false;
  for (const auto& el : current_waypoints_)
  {
    if (getPlaneDistance(el.pose.pose.position, current_pose_.position)
      > minimum_lookahead_distance_)
    {
      is_valid_curve = true;
      break;
    }
  }
  if (!is_valid_curve)
  {
    return false;
  }
  // if is_linear_interpolation_ is false or next waypoint is first or last
  if (!is_linear_interpolation_ || next_waypoint_number_ == 0 ||
    next_waypoint_number_ == (static_cast<int>(current_waypoints_.size() - 1)))
  {
    next_target_position_ =
      current_waypoints_.at(next_waypoint_number_).pose.pose.position;
    *output_kappa = calcCurvature(next_target_position_);
    return true;
  }

  // linear interpolation and calculate angular velocity
  bool interpolation =
    interpolateNextTarget(next_waypoint_number_, &next_target_position_);

  if (!interpolation)
  {
    ROS_INFO_STREAM("lost target! ");
    return false;
  }

  // ROS_INFO("next_target : ( %lf , %lf , %lf)",
  //  next_target.x, next_target.y,next_target.z);

  *output_kappa = calcCurvature(next_target_position_);
  return true;
}
```

作用是计算路径曲率并检查路径是否有效

- `getNextWaypoint()`：调用函数 `getNextWaypoint()` 来获取下一个路径点。

- `if (next_waypoint_number_ == -1)`：检查变量 `next_waypoint_number_` 是否为 -1，如果是，则输出错误信息并返回 `false`，表示丢失了下一个路径点。

- 接下来通过循环遍历当前路径点列表 `current_waypoints_`，对每个路径点执行以下操作：
  - `getPlaneDistance(el.pose.pose.position, current_pose_.position)`：计算当前车辆位置 `current_pose_` 和路径点位置 `el.pose.pose.position` 之间的平面距离。
  - 检查平面距离是否大于预设的最小前瞻距离 `minimum_lookahead_distance_`。
  - 如果有任何一个路径点的平面距离大于最小前瞻距离，则将 `is_valid_curve` 设为 `true`，并跳出循环。

- 如果 `is_valid_curve` 为 `false`，表示找不到有效的路径曲率，返回 `false`。


- 如果 `is_linear_interpolation_` 为 `false`，或者下一个路径点是路径的第一个或最后一个点，则执行以下操作：
  - 将 `next_target_position_` 设置为当前路径列表中下一个路径点的位置。
  - 调用 `calcCurvature()` 函数计算路径曲率，并将结果存储到 `output_kappa` 指针指向的位置。
  - 返回 `true` 表示成功获取了路径曲率。
- 如果 `is_linear_interpolation_` 为 `true`，并且下一个路径点不是路径的第一个或最后一个点，则调用 `interpolateNextTarget()` 函数进行线性插值，根据插值结果获取下一个目标位置。



## pure_pursuit_core.cpp

### 总览

- **订阅话题（输入）**

  ```c++
  sub1_ = nh_.subscribe("final_waypoints", 10,
  &PurePursuitNode::callbackFromWayPoints, this);
  sub2_ = nh_.subscribe("current_pose", 10,
  &PurePursuitNode::callbackFromCurrentPose, this);
  sub3_ = nh_.subscribe("config/waypoint_follower", 10,
  &PurePursuitNode::callbackFromConfig, this);
  sub4_ = nh_.subscribe("current_velocity", 10,
  &PurePursuitNode::callbackFromCurrentVelocity, this);
  ```

  包含了由规划模块输出的waypoints，以及当前车辆的位置current_pose与速度current_velocity

- **发布话题（输出）**

  ```c++
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  pub2_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 10);
  pub3_ = nh_.advertise<tablet_socket_msgs::gear_cmd>("gear_cmd", 0);
  ```

  为控制信号ctrl_cmd，包括车辆的线速度、线加速度与转向角；twist_raw应该是用于仿真中车辆的控制

  - 剩下一些均为在rviz中可视化的内容

    ```c++
    pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
    pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
    pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
    // debug tool
    pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);
    pub15_ =
    nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
    pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
    pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
    pub18_ =C
    nh_.advertise<visualization_msgs::Marker>("expanded_waypoints_mark", 0);
    ```




### 1.PurePursuitNode构造函数

```c++
PurePursuitNode::PurePursuitNode()
  : private_nh_("~")
  , pp_()
  , LOOP_RATE_(30)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , is_velocity_set_(false)
  , current_linear_velocity_(0)
  , command_linear_velocity_(0)
  , direction_(LaneDirection::Forward)
  , velocity_source_(-1)
  , const_lookahead_distance_(4.0)
  , const_velocity_(5.0)
  , lookahead_distance_ratio_(2.0)
  , minimum_lookahead_distance_(6.0)
{
  initForROS();
  health_checker_ptr_ =
    std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);
  health_checker_ptr_->ENABLE();
  // initialize for PurePursuit
  pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}
```

- `private_nh_("~")`：使用私有命名空间初始化私有节点句柄 `private_nh_`。这通常用于从ROS参数服务器获取私有参数。

- `pp_()`：使用默认构造函数初始化成员变量 `pp_`，这是 `PurePursuit` 类的一个实例，用于执行纯追踪算法。

- **`LOOP_RATE_(30)`：初始化循环频率 `LOOP_RATE_` 为 30 Hz，这是控制节点主循环执行速率的变量。**

- `is_waypoint_set_(false)`、`is_pose_set_(false)`、`is_velocity_set_(false)`：初始化布尔类型的标志位，用于标识路径点、车辆姿态和车辆速度是否已设置，默认都为 `false`。

- `current_linear_velocity_(0)`、`command_linear_velocity_(0)`：初始化当前线性速度和命令线性速度为零。

- `direction_(LaneDirection::Forward)`：初始化车道方向为前向。

- `velocity_source_(-1)`：初始化速度来源，默认为 -1。

  此时由来见pure_pursuit_core.h代码中

  ```c++
  enum class Mode : int32_t
  {
    waypoint,
    dialog,
  
    unknown = -1,
  };
  
  template <class T>
  typename std::underlying_type<T>::type enumToInteger(T t)
  {
    return static_cast<typename std::underlying_type<T>::type>(t);
  }
  ```

  - `enum class Mode : int32_t`: 这定义了一个枚举类 `Mode`，它的底层类型（underlying type）是 `int32_t`。
  - `waypoint` 和 `dialog`: 这是枚举类 `Mode` 中的枚举值。它们分别被分配了默认的整数值（`waypoint` 默认为 0，`dialog` 默认为 1）
  - `unknown = -1`: 这是另一个枚举值 `unknown`，并且手动指定了它的值为 -1
  - 模板函数 `enumToInteger`，它接受一个枚举类型的参数 `t`；使用了 `static_cast` 将枚举值 `t` 转换为其底层类型的整数值，并返回该值

- **`const_lookahead_distance_(4.0)`：初始化常量前瞻距离为 4.0 米。**
- **`const_velocity_(5.0)`：初始化常量速度为 5.0 米/秒。**
- **`lookahead_distance_ratio_(2.0)`：初始化前瞻距离比例为 2.0。**
- **`minimum_lookahead_distance_(6.0)`：初始化最小前瞻距离为 6.0 米。**
- `initForROS()`：调用 `initForROS()` 函数，该函数用于ROS相关的初始化，可能包括创建订阅器、发布器、服务等。
- `health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_)`：使用 `std::make_shared` 创建一个名为 `health_checker_ptr_` 的共享指针，指向一个健康检查器对象，该对象使用节点句柄 `nh_` 和私有节点句柄 `private_nh_` 进行初始化。
- `health_checker_ptr_->ENABLE()`：启用健康检查器。
- `pp_.setLinearInterpolationParameter(is_linear_interpolation_)`：调用 `setLinearInterpolationParameter()` 函数，将线性插值参数设置为 `is_linear_interpolation_` 的值。



### 2.函数`computeLookaheadDistance`

```c++
double PurePursuitNode::computeLookaheadDistance() const
{
  if (velocity_source_ == enumToInteger(Mode::dialog))
  {
    return const_lookahead_distance_;
  }

  double maximum_lookahead_distance = current_linear_velocity_ * 10;
  double ld = current_linear_velocity_ * lookahead_distance_ratio_;

  if(direction_ == LaneDirection::Backward){
    ld =  ld < minimum_reverse_lookahead_distance_ ? minimum_reverse_lookahead_distance_ :
      ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
  }else{
    ld =  ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ :
      ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
  }
  std::cout<<"look ahead distance :"<<ld<<std::endl;
  return ld;
}
```

用于计算纯追踪算法中的前瞻距离（lookahead distance）

- **固定前瞻距离模式**：首先检查`velocity_source_`的值，如果它等于`Mode::dialog`（枚举类型，表示速度来源的模式），则返回一个固定的前瞻距离`const_lookahead_distance_`。这种模式适用于某些特定场景，比如速度和前瞻距离都是由用户通过对话框直接设置的。

- **基于速度的动态前瞻距离**：
  - **计算一个基于当前线性速度`current_linear_velocity_`和前瞻距离比例`lookahead_distance_ratio_`的前瞻距离`ld`。**
  - 设置一个最大前瞻距离，为当前线性速度的10倍。
  - 如果车辆当前方向为后退（`LaneDirection::Backward`），则前瞻距离`ld`会被调整至不小于`minimum_reverse_lookahead_distance_`且不大于`maximum_lookahead_distance`。
  - 如果车辆方向为前进，同样地，`ld`会被调整至不小于`minimum_lookahead_distance_`且不大于`maximum_lookahead_distance`。



### 3.函数`publishDeviationCurrentPosition`

```c++
void PurePursuitNode::publishDeviationCurrentPosition(
  const geometry_msgs::Point& point,
  const std::vector<autoware_msgs::Waypoint>& waypoints) const
{
  // Calculate the deviation of current position
  // from the waypoint approximate line

  if (waypoints.size() < 3)
  {
    return;
  }

  double a, b, c;
  getLinearEquation(
    waypoints.at(2).pose.pose.position, waypoints.at(1).pose.pose.position,
    &a, &b, &c);

  std_msgs::Float32 msg;
  msg.data = getDistanceBetweenLineAndPoint(point, a, b, c);

  pub17_.publish(msg);
}
```

发布当前位置相对于路径近似直线的偏差

- 函数接受两个参数：
  - `point`：当前位置的几何坐标点。
  - `waypoints`：包含一系列路径点的向量。

- 逻辑如下：
  - 检查路径点数量是否小于 3，如果是，则直接返回，因为至少需要三个路径点才能构成一条直线。
  - 计算通过第二个和第一个路径点的线段的线性方程，得到直线的参数 a、b、c。
  - 使用当前位置与计算得到的直线方程，计算当前位置到路径直线的距离。 



### 4.回调函数`callbackFromWayPoints`

```c++
void PurePursuitNode::callbackFromWayPoints(
  const autoware_msgs::LaneConstPtr& msg)
{
  command_linear_velocity_ =
    (!msg->waypoints.empty()) ? msg->waypoints.at(0).twist.twist.linear.x : 0;
  if (add_virtual_end_waypoints_)
  {
    const LaneDirection solved_dir = getLaneDirection(*msg);
    direction_ = (solved_dir != LaneDirection::Error) ? solved_dir : direction_;
    autoware_msgs::Lane expanded_lane(*msg);
    expand_size_ = -expanded_lane.waypoints.size();
    connectVirtualLastWaypoints(&expanded_lane, direction_);
    expand_size_ += expanded_lane.waypoints.size();

    pp_.setCurrentWaypoints(expanded_lane.waypoints);
  }
  else
  {
    pp_.setCurrentWaypoints(msg->waypoints);
  }
  is_waypoint_set_ = true;
}
```

- **命令线性速度更新**：
  - 首先检查接收到的路径点（`msg->waypoints`）是否为空。如果不为空，将命令线性速度（`command_linear_velocity_`）设置为路径中第一个点的线性速度值（`msg->waypoints.at(0).twist.twist.linear.x`）。如果路径点为空，命令线性速度被设置为0。

- **是否添加虚拟终点路径点**`add_virtual_end_waypoints_`

  -  **添加：**

    根据路径点消息`msg`中的信息，计算行驶方向（`solved_dir`）。如果解算的方向不是`LaneDirection::Error`，则更新`direction_`成员变量

    在添加虚拟路径点之前，记录扩展前的路径点数量。通过调用`connectVirtualLastWaypoints`函数，为`expanded_lane`添加虚拟的最后路径点，这是为了在路径结束处提供更平滑的转换或为路径规划提供额外的信息。

  - **不添加：**

    直接使用消息`msg`中的原始路径点更新`PurePursuit`对象的当前路径点列表

- **更新路径点设置标志**：将`is_waypoint_set_`标志设置为`true`，表示路径点已经被成功设置



### 5.函数`convertCurvatureToSteeringAngle`

```c++
double convertCurvatureToSteeringAngle(
  const double& wheel_base, const double& kappa)
{
  return atan(wheel_base * kappa);
}
```

根据给定的车轮轴距（`wheel_base`）和曲率（`kappa`）所需的转向角度

**可以知道该车辆模型为：基于车辆后轴中心的单车模型**



### 6.函数`publishGearCmd`

```
void PurePursuitNode::publishGearCmd(const int &gear_target){
  tablet_socket_msgs::gear_cmd gear_msg;
  gear_msg.gear = gear_target;
  pub3_.publish(gear_msg);
}
```

用于发布传动命令

- 接受一个参数`gear_target`：传动目标值，即希望车辆切换到的传动档位

- 将传动目标值 `gear_target` 赋值给 `gear_msg` 的 `gear` 成员变量
- **发布控制命令**：最后，通过`pub3_`发布器发布传动命令消息



### 7.函数`publishControlCommandStamped`

```C++
void PurePursuitNode::publishControlCommandStamped(
  const bool& can_get_curvature, const double& kappa) const
{
  if (!publishes_for_steering_robot_)
  {
    return;
  }

  autoware_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
  ccs.cmd.linear_acceleration = can_get_curvature ? computeCommandAccel() : 0;
  ccs.cmd.steering_angle =
    can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;

  pub2_.publish(ccs);
}
```

- **检查是否发布转向控制命令**：首先检查`publishes_for_steering_robot_`标志。如果这个标志被设置为`false`，表示当前节点不需要发布转向相关的控制命令，函数将直接返回，不执行任何操作

- **创建控制命令消息**：创建一个`autoware_msgs::ControlCommandStamped`消息`ccs`，用于携带控制命令。`ControlCommandStamped`消息包括时间戳、线性速度、线性加速度和转向角度等字段
- **计算并设置控制命令**：
  - **线性速度**：如果可以从`PurePursuit`实例获取曲率(`can_get_curvature`为`true`)，则调用`computeCommandVelocity`方法计算线性速度；否则设置为0。
  - **线性加速度**：同样，如果可以获取曲率，调用`computeCommandAccel`方法计算线性加速度；否则设置为0。
  - **转向角度**：如果可以获取曲率，调用`convertCurvatureToSteeringAngle`方法将曲率转换为转向角度；否则设置为0。这个转换考虑了车辆的轮距(`wheel_base_`)。

- **发布控制命令**：最后，通过`pub2_`发布器发布控制命令消息。



### 8.函数`publishTwistStamped`

```c++
void PurePursuitNode::publishTwistStamped(
  const bool& can_get_curvature, const double& kappa) const
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
  ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
  pub1_.publish(ts);
}
```

发布速度指令的另一个函数，这次使用的是`geometry_msgs::TwistStamped`消息类型。该方法基于当前的曲率（`kappa`）来决定车辆的线性速度和角速度

- **创建速度指令消息**：首先，创建一个`geometry_msgs::TwistStamped`消息`ts`，用于携带线性和角速度指令。`TwistStamped`消息包含时间戳和一个`Twist`消息，后者进一步包含线性和角速度向量。
- **计算并设置速度指令**：
  - **线性速度**（`ts.twist.linear.x`）：如果能从当前的路径信息中获取有效的曲率（即`can_get_curvature`为`true`），则调用`computeCommandVelocity`方法计算线性速度；否则，线性速度设置为0。
  - **角速度**（`ts.twist.angular.z`）：如果能获取有效的曲率，那么角速度设置为曲率`kappa`乘以计算出的线性速度`ts.twist.linear.x`；否则，角速度设置为0。



### 主函数`run`

```c++
void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!is_pose_set_ || !is_waypoint_set_ || !is_velocity_set_)
    {
      ROS_WARN("Necessary topics are not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }

    pp_.setLookaheadDistance(computeLookaheadDistance());
    pp_.setMinimumLookaheadDistance(minimum_lookahead_distance_);
    pp_.setMinimumReverseLookaheadDistance(minimum_reverse_lookahead_distance_);
    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa);

    publishTwistStamped(can_get_curvature, kappa);
    publishControlCommandStamped(can_get_curvature, kappa);
    health_checker_ptr_->NODE_ACTIVATE();
    health_checker_ptr_->CHECK_RATE("topic_rate_vehicle_cmd_slow", 8, 5, 1,
      "topic vehicle_cmd publish rate slow.");
    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(
      pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
    pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));
    pub15_.publish(displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(
          pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));
    if (add_virtual_end_waypoints_)
    {
      pub18_.publish(
        displayExpandWaypoints(pp_.getCurrentWaypoints(), expand_size_));
    }
    std_msgs::Float32 angular_gravity_msg;
    angular_gravity_msg.data =
      computeAngularGravity(computeCommandVelocity(), kappa);
    pub16_.publish(angular_gravity_msg);

    publishDeviationCurrentPosition(
      pp_.getCurrentPose().position, pp_.getCurrentWaypoints());
    
    if(direction_ == LaneDirection::Forward){
      GEAR_ = 4;
    }else if(direction_ == LaneDirection::Backward){
      GEAR_ = 2;
    }
    publishGearCmd(GEAR_);

    is_pose_set_ = false;
    is_velocity_set_ = false;
    is_waypoint_set_ = false;

    loop_rate.sleep();
  }
}
```

作为PurePursuitNode类的主循环逻辑

**关系到未来可能会修改和调试的内容：**

- **循环频率设置**：通过`ros::Rate`对象设置循环的执行频率，`LOOP_RATE_`定义了每秒循环的次数

  这里根据参数设置是30

- `is_pose_set_`、`is_waypoint_set_`、`is_velocity_set_`三者都为True才能执行后续指令

  三个都在回调函数中被设置

- 档位变换

  ```c++
  if(direction_ == LaneDirection::Forward){
  GEAR_ = 4;
  }else if(direction_ == LaneDirection::Backward){
  GEAR_ = 2;
  }
  ```

  这里的档位数字是有明确含义的，这里的档位4就是D档，2就是R档

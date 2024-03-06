# 2024.3.4

## 已完成内容

1. CAN连通
2. 能够把CAN报文转换成为ROS消息
3. report上传部分数据流流通



## 笔记

- CAN笔记

  ==补充路径？？？？==

  - 启动CAN

    ```
    ./start
    ```

  - 打印车辆CAN数据流

    ```
    candump can1
    ```

- 代码修改笔记

  1. **在一个class类中**，进行数据的输入(subscribe)时，要指定回调函数的地址的类，比如

  ```c++
  sub_control_command = nh.subscribe("/vehicle_cmd", 1, &ControlConverter::control_command_callback,this);
  ```

  其中**`control_command_callback`为该ControlConverter类中的一个成员函数**，所以在使用时需要加入`&ControlConverter::`

  并且在这种情况下，在**定义`control_command_callback`回调函数时不需要重复声明**，例如

  ```cc
  void ControlConverter::control_command_callback(const autoware_msgs::VehicleCmd::ConstPtr &msg)
  ```

  其中**`ControlConverter::control_command_callback`即为重复声明，需去掉类声明，修改为如下**

  ```cc
  void control_command_callback(const autoware_msgs::VehicleCmd::ConstPtr &msg)
  ```

  即可

  

  2. **指针访问对象成员方式**

  ```cc
  void control_command_callback(const autoware_msgs::VehicleCmd::ConstPtr &msg)
  {
        speed = fabs(msg->ctrl_cmd.linear_velocity);
        steer = -500 * (msg->ctrl_cmd.steering_angle / 0.43);
        ros::Time stamp = ros::Time::now();
  ...
  }
  ```

  其中，`msg` 是一个指向 `autoware_msgs::VehicleCmd` 类型对象的指针，所以`msg->ctrl_cmd.linear_velocity` 表示访问 `VehicleCmd` 对象中 `ctrl_cmd` 成员变量的 `linear_velocity` 属性

  **`msg` 是一个 `autoware_msgs::VehicleCmd` 类型的常量指针，因此使用 `->` 来访问它的成员，而不是使用 `.`。**
  
  
  
  3. **节点对应依赖**
  
  在`CMakeLists.txt`文件中，**每一个节点对应的依赖需要即使添加，即`一个节点名称`对应`一个dependencies和一个libraries`**，示例如下：
  
  ```cmake
  add_executable(${PROJECT_NAME}_command_node 
  src/ADCU_BrakeCmd_111.cc
  src/ADCU_SteerCmd_113.cc
  src/ADCU_ParkCmd_112.cc
  src/ADCU_BodyCmd_115.cc
  src/ADCU_DriveCmd_114.cc
  src/ADCU_PowerCmd_117.cc
  src/ADCU_CldDrvCmd_118.cc
  src/ADCU_CldBodyCmd_119.cc
  src/ADCU_CldPowerCmd_11A.cc
  src/ADCU_CrashClrCmd_12A.cc
  src/ADCU_TripClear_253.cc
  src/PCU_PowerCmd_127.cc
  src/ADCU_SweepCmd_151.cc
  src/Byte.cc
  src/command_node.cc
  )
  ```
  
  对应依赖为
  
  ```cmake
  add_dependencies(${PROJECT_NAME}_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  ## Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}_command_node
     ${catkin_LIBRARIES}
   )
  ```
  
  和
  
  ```cmake
  add_executable(${PROJECT_NAME}_control_converter
  src/control_command_speed_exercise.cpp
  )
  ```
  
  对应依赖为
  
  ```cmake
  add_dependencies(${PROJECT_NAME}_control_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  ## Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}_control_converter
     ${catkin_LIBRARIES}
   )
  ```
  



## 需要修改的地方


1. 修改raw_vehicle_command_converter下的node.hpp和node.cpp文件，主要包括信号名称等内容

2. 修改代码，打通control_converter和command_node的数据流

   **方法：通过启动`e_car_main_command_node`和`e_car_main_control_converter`两个节点后，通过rqt_graph图形可视化来判断数据流**



# 2024.3.5

## 已完成内容

1. report报文部分数据流全部流通，能够正确接收到车辆实际的速度、转角等信息



## 遇到的问题

手动给定速度、转角等控制输入，消息接收端部分正确，部分错误

初步估计为代码逻辑问题，主要集中在`/vehicle_cmd`的回调函数`control_command_callback`部分

 

## 需要修改的地方

1. 根据使用手册说明要求，加入power_cmd的定义部分，即开启自动驾驶的电源部分
1. 先将有关清扫逻辑的内容注释掉，集中解决关于速度、转角的内容，防止清扫逻辑的错误对其造成影响
1. body_cmd和power_cmd未有效发布话题，即`nh.advertise()`和`××.publish()`没有一一对应



# 2024.3.7

# ROS文件系统（可结合wiki官网）

## 创建ROS软件包

###catkin软件包组成

一个包要想称为catkin软件包，必须符合以下要求：

- 这个包必须有一个符合catkin规范的package.xml文件
  - 这个`package.xml`文件提供有关该软件包的元信息
- 这个包必须有一个catkin版本的CMakeLists.txt文件
  - 如果它是个Catkin元包的话，则需要有一个`CMakeList.txt`文件的相关样板
- 每个包必须有自己的目录
  - 这意味着在同一个目录下不能有嵌套的或者多个软件包存在

最简单的软件包看起来就像这样：

```c++
my_package/
  CMakeLists.txt
  package.xml
```



### 创建catkin工作空间

```c++
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```



###创建功能包

```c++
$ cd ~/catkin_ws/src
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]//其中depend为依赖
```

例如：

```c++
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

会**在catkin_ws的src目录下创建一个名为beginner_tutorials的文件夹**，这个文件夹里面包含**一个include目录，一个src目录，一个package.xml文件和一个CMakeLists.txt文件**，两个文件都已经部分填写了你在执行catkin_create_pkg命令时提供的信息。

**所有源码**必须放入**功能包中**中进行编译，其中**src目录下一般放置cpp文件**之类的功能包代码的，**include目录下放置一些比如.h的头文件**

*同一工作空间下，不能存在同名功能包*



#### 一些可能会用到的依赖包

```c++
cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
```



####编译catkin工作空间

```c++
$ cd ~/catkin_ws/
$ catkin_make
```

此时**产生devel和build两个目录**，在devel文件夹里面可以看到几个`setup.sh`文件，是开发空间的默认位置；build目录是构建空间的默认位置，同时cmake和make也是在这里被调用来配置和构建软件包。source文件中的任何一个都可以将当前工作空间设置在环境的最顶层。在`~/.bashrc`中最后source的`setup.bash`将是最顶层的工作空间。

```c++
$ catkin_make install
```

此时才会产生intsall文件夹



####设置catkin环境变量

```c++
$ source devel/setup.bash //非常重要
//可通过 $ echo $ROS_PACKAGE_PATH 查找ROS功能包的路径
```



####创建自定义msg类型功能包及后续连带操作（可参考http://wiki.ros.org/msg）

可创建一个空的package单独存放msg类型文件，并在其中创建的msg文件夹下新建一个名为`××.msg`消息类型文件（比如`Test.msg`）
.msg文件由两部分组成：**字段**和**常量**。字段是消息内部发送的数据，常量定义可用于解释这些字段的有用值（例如，整数值的类似枚举常量）。

- **字段**：字段类型 字段名称
  - **字段类型**：
  
    int8取值范围是：-128 — 127

    int16 意思是16位整数(16bit integer)，相当于short 占2个字节 -32768 — 32767
  
    int32 意思是32位整数(32bit integer), 相当于 int 占4个字节 -2147483648 —2147483647
  
    int64 意思是64位整数(64bit interger), 相当于 long long 占8个字节 -9223372036854775808 — 9223372036854775807  
    
  - ![image-20231105223126645](C:\Users\李文博\AppData\Roaming\Typora\typora-user-images\image-20231105223126645.png)
  
    
  
  - **字段名称**：限制为字母字符，后跟字母数字和下划线的任意混合
  
  - **标头**：**.msg文件的第一个字段为：Header header 必须添加，否则报错**
  
- 常量：常量类型 常量名称=常量值

例如：int32 X=123；string FOO=foo 

比如Test.msg的内容如下：

```msg
float32[] data
float32 vel
geometry_msgs/Pose pose
string name
```



##### 修改package.xml文件

需要`message_generation`生成C++或Python能使用的代码，需要`message_runtime`提供运行时的支持，所以package.xml中添加以下**两句**：

```xml
<build_depend> message_generation </build_depend>
<exec_depend> message_runtime </exec_depend>
```



##### 修改CMakeLists.txt文件

- 首先调用`find_package`查找依赖的包，必备的有`roscpp、rospy、message_generation`，**其他根据具体类型添加**，比如上面的msg文件中用到了`geometry_msgs/Pose pose`类型，那么必须查找`geometry_msgs`

  ```cmake
  find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs geometry_msgs)
  ```

- 然后是`add_message_files`，指定msg文件

```cmake
add_message_files(
  FILES
  Test.msg
  # Message2.msg
)
```

- 然后是`generate_messages`，指定生成消息文件时的依赖项，比如上面嵌套了其他消息类型`geometry_msgs`，那么必须注明

```cmake
#generate_messages必须在catkin_package前面
generate_messages(
 DEPENDENCIES
 geometry_msgs
)
```

- 然后是`catkin_package`设置运行依赖

```cmake
catkin_package(
CATKIN_DEPENDS message_runtime
)
```



####理解package.xml文件（xml为可拓展标记语言）√

```xml
<?xml version="1.0"?>
<package format="2">
  <name>pix_driver</name>//功能包名字
  <version>0.0.0</version>//版本号
  <description>The pix_driver package</description>//功能包的描述信息

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="team1@todo.todo">team1</maintainer>//维护者的email地址信息

    
  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/pix_driver</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
    //功能包的依赖
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>autoware_msgs</build_depend>
  <build_depend>can_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>pix_driver_msgs</build_depend>
    
  <build_export_depend>autoware_msgs</build_export_depend>
  <build_export_depend>can_msgs</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>pix_driver_msgs</build_export_depend>
    
  <exec_depend>autoware_msgs</exec_depend>
  <exec_depend>can_msgs</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>pix_driver_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```



####理解CMakeLists.txt文件（用于编译的cmake规则列表）√

CMake是一个**跨平台的编译 (Build)工具**，可以用简单的语句来描述所有平台的编译过程。 该文件用于编译在 ROS 中编写的程序的命令，还具有将源代码和其他文件转换为可执行文件（即您的计算机可以运行的代码）的命令。

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(pix_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(autoware_msgs_DIR ~/autoware.gf2/autoware-1.14/install/autoware_msgs/share/autoware_msgs/cmake/)
set(autoware_vehicle_msgs_DIR /home/t/ros_driver/catkin_ws_msg/devel/share/autoware_vehicle_msgs/cmake/)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
//找到依赖的功能包
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  can_msgs
  roscpp
  std_msgs
  pix_driver_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   autoware_msgs#   can_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pix_driver
 CATKIN_DEPENDS autoware_msgs can_msgs roscpp std_msgs pix_driver_msgs
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pix_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
//把代码编译成可执行文件
add_executable(${PROJECT_NAME}_report_node 
src/report_node.cc
src/steering_report_502.cc
src/Byte.cc
src/gear_report_503.cc
src/bms_report_512.cc
src/brake_report_501.cc
src/park_report_504.cc
src/throttle_report_500.cc
src/vcu_report_505.cc
)

add_executable(${PROJECT_NAME}_command_node
src/command_node.cc
src/brake_command_101.cc
src/gear_command_103.cc
src/park_command_104.cc
src/steering_command_102.cc
src/throttle_command_100.cc
src/vehicle_mode_command_105.cc
src/acu_sweepctrlcmd_107.cc
src/Byte.cc
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

//add_dependencies（）为添加依赖项，

add_dependencies(${PROJECT_NAME}_report_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_command_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

//target_link_libraries（），用于代码的可执行文件的链接，其中${catkin_LIBRARIES}为ROS基本库

target_link_libraries(${PROJECT_NAME}_report_node
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_command_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}_report_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_command_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pix_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```



## ROS中计算程序执行时间

- 时间格式

分为**时刻（Time）**和**持续时间（Duration）**，且分为秒（s）和纳秒（ns），换算关系为：`1nsec=1e-9sec`

```c++
int32 sec
int32 nsec
```

- ros::Time::now()

检索当前时间，是**时刻**时间

- ros::Duration

代表**持续的一段**时间，可以是复数

- toSec()

将“1 ros时间格式说明”中所表示的格式转为秒（s）

- toNSec()

将时间或时间戳转为纳秒（ns）



## 理解ROS节点

只不过是ROS软件包中的一个**可执行文件**。ROS节点使用ROS客户端库与其他节点通信，节点可以发布或订阅话题，也可以提供或使用服务。

- roscore=ros+core，是运行所有ROS程序前首先要运行的命令（注：保证**有一个roscore在运行**就够了）
- rospy = Python客户端库
- roscpp = C++客户端库
- rosnode=ros+node，显示当前**正在运行**的ROS节点信息（注：要**保持以前的终端开着**）
- rosout用于收集和记录节点的调试输出，所以总是在运行的
- rosrun=ros+run，可以用包名直接运行软件包内的节点（而不需要知道包的路径）



## 理解ROS话题

节点与节点之间是通过一个**ROS话题来相互通信**的。

rostopic命令工具可**获取ROS话题的信息**

rostopic pub可以把数据发布到当前某个正在广播的话题上

用法：

```c++
rostopic pub [topic] [msg_type] [args]
```

更多内容可以参考[ROSwiki](http://wiki.ros.org/cn/ROS/Tutorials/UnderstandingTopics#ROS.2Bi92YmA-)

### ROS消息

话题的通信是通过**节点间发送ROS消息**实现的



## ROS中发布者Publisher的编程实现（C++）

### 1.必须包含的**头文件**

`include "ros/ros.h"`

###2.具体代码示例（可结合对应[wiki](http://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)）

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"//引用了位于std_msgs包里的std_msgs/String消息
#include <sstream>//用于字符串流操作

int main(int argc, char **argv)
{
  
  // ROS节点初始化
  ros::init(argc, argv, "talker");

  // 创建节点句柄，管理节点资源
  ros::NodeHandle n;

  // 创建一个Publisher，发布名为chatter的主题，消息类型为std_msgs::String，队列长度1000
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);//n为上方句柄；advertise为ROS自带的标准函数，将一个Publisher与特定的消息类型和主题关联起来，使得你的节点能够发布该类型的消息到该主题上
    
  // 设置循环的频率，此处为10Hz
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())//roscpp将安装一个SIGINT处理程序，它能够处理Ctrl+C操作，让ros::ok()返回false
  {
   //初始化std_msgs::String类型的消息，基于String.msg文件中的定义去进行编译
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);//发布者把这个信息广播给了任何已连接的节点

    ros::spinOnce();//用来接收回调函数

    loop_rate.sleep();//按照循环频率延时
    ++count;
  }

  return 0;
}
```
**总结：如何实现一个发布者**

- 初始化ROS节点
- 向ROS Master注册节点信息，包括发布的话题名和话题中的消息类型
- 创建消息数据
- 按照一定频率循环发布消息



### 3.函数使用：

`ros::Publisher turtle_vel_pub = n.advertise<消息类型>( “发布话题的名字”, 消息缓冲队列大小, bool latch = false )`

当**latch为默认值=false**时，表示**最新发布的消息不会被保持在主题上**，新的订阅者只有在消息发布时才会接收到消息，而不会立即接收到最新的消息。在这种情况下，新的订阅者只有在有新的消息发布到 `/turtle1/cmd_vel` 主题时才会接收到消息。

当latch=true时，表示**消息将被持久化**，新的订阅者在连接到主题时会立即接收到最新的消息。

***一般来说，对于静态地图为true，如果没有给出初始地图则为false。***



## ROS中订阅者Subscriber的编程实现(C++)

## 1.必须包含的**头文件**

`include "ros/ros.h"`

###2.具体代码示例（可结合对应[wiki](http://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)）

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

//回调函数，当有新消息到达chatter话题时它就会被调用
void chatterCallback(const std_msgs::String::ConstPtr& msg)//长指针
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());//将接收到的信息打印出来，此处的data应为数据流
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber chatter_sub = n.subscribe("chatter", 1000, chatterCallback);//通过主节点订阅chatter话题，ROS将调用chatterCallback()函数，1000为队列大小

  ros::spin();//循环等待回调函数

  return 0;
}
```

**总结：如何实现一个订阅者**

- 初始化ROS节点
- 订阅需要的话题
- 循环等待话题消息，接收到消息后进入回调函数
- 在回调函数中完成消息处理

























## ROS变量初始化大体框架

```python
class ×× ：
    def __init__(self):
        self.speed = 0.0
        self.gear_act = 3
        self.timer = rospy.Time.now().to_sec()  # 获取当前系统时间，并转换为秒数
        self.stop_for_shift = False
        ......
```

等于

```c++
class ×× 
{
 private://成员变量声明
    double speed;
    int gear_act;
    double timer; // 获取当前系统时间，并转换为秒数
    bool stop_for_shift;
    ......
    
 public:
    ××()//创建构造函数
    {
    speed=0.0;
    gear_act=3;,
    timer=ros::Time::now().toSec();// 获取当前系统时间，并转换为秒数
    stop_for_shift=false,
    ......
    }    
}     
```



# Python与C++编译区别

## 话题订阅

举例：

- python下：

```python
from pix_driver_msgs.msg import GearReport
def __init__(self): #前提
self.sub_gear_report = rospy.Subscriber('/pix/gear_report', GearReport, self.gear_report_callback)
def gear_report_callback(self, msg):
```

- C++下：

```c++
#include "pix_driver_msgs/GearReport.h"
ros::Subscriber sub_gear_report; //前提
ros::NodeHandle nh; //前提
sub_gear_report= nh.subscribe("/pix/gear_report", 1, &ControlConverter::gear_report_callback,this);
void ControlConverter::gear_report_callback(const pix_driver_msgs::GearReport::ConstPtr &msg)
```

- ==区别==：

  - **消息类型**（GearReport）**放的位置不同**

    -  原因：**在C++中**，当你使用`&ControlConverter::gear_report_callback`时，编译器知道这是一个`ControlConverter`类的成员函数，因此可以根据函数签名推断消息类型，因此在订阅主题时**可以省略消息类型**。

      ​            **在python中**，由于动态类型和运行时类型推断的特性，需要**显式地提供消息类型**，即`GearReport`，以便在运行时进行正确的订阅。

  - **ConstPtr的用法**

    - 在ROS中， `ConstPtr`是一个**智能指针**类型，指向常量的消息对象。此处表示一个常量引用，该引用指向`pix_driver_msgs::GearReport`的消息类型。


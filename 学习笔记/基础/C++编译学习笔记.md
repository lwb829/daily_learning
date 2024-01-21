# C＋＋语言编译（结合[google代码规范](https://zh-google-styleguide.readthedocs.io/en/stable/google-cpp-styleguide/contents/)）

##命名约定

### 通用命名规则

尽可能使用描述性的命名，让代码易于新读者理解更重要。不要用只有项目开发者能理解的缩写， 也不要通过砍掉几个字母来缩写单词。

```c++
int price_count_reader;    // 无缩写
int num_errors;            // "num" 是一个常见的写法
int num_dns_connections;   // 人人都知道 "DNS" 是什么

int n;                     // 毫无意义.
int nerr;                  // 含糊不清的缩写.
int n_comp_conns;          // 含糊不清的缩写.
int wgc_connections;       // 只有贵团队知道是什么意思.
int pc_reader;             // "pc" 有太多可能的解释了.
int cstmr_id;              // 删减了若干字母.
```

### 文件命名

一般全部以小写为主，可以包含下划线（__）或连字符（-）

```cc
my_useful_class.cc
my-useful-class.cc
myusefulclass.cc

myusefulclass_test.cc // _unittest 和 _regtest 已弃用.
```

### 类型命名

类型名称的每个单词首字母均大写， 不包含下划线：MyExcitingClass，MyExcitingEnum。

所有类型命名 —— 类，结构体，类型定义 (typedef)，枚举，类型模板参数 —— 均使用相同约定，即**以大写字母开始**，每个单词首字母均大写，不包含下划线。例如:

```c++
// 类和结构体
class UrlTable { ...
class UrlTableTester { ...
struct UrlTableProperties { ...

// 类型定义——给数据类型取别名
typedef hash_map<UrlTableProperties *, string> PropertiesMap;

// using 别名
using PropertiesMap = hash_map<UrlTableProperties *, string>;

// 枚举
enum UrlTableErrors { ...
```

### 变量命名

- 变量 (包括函数参数) 和数据成员名**一律小写**，单词之间**用下划线连接**。

```c++
string table_name;  // 好 - 用下划线.
string tablename;   // 好 - 全小写.
string tableName;  // 差 - 混合大小写
```

- **类**的成员变量**以下划线结尾**，但结构体的就不用。

```c++
class TableInfo //类
{
...
private:
string table_name_;  // 好 - 后加下划线.
string tablename_;   // 好.
static Pool<TableInfo>* pool_;  // 好.
};
```

```c++
struct UrlTableProperties //结构体
{
string name;
int num_entries;
static Pool<UrlTableProperties>* pool;
};
```

### 函数命名

#### 总述

常规函数使用**大小写混合**，取值和设值函数则要求**与变量名匹配**：MyExcitingFunction()，MyExcitingMethod()，my_exciting_member_variable()，set_my_exciting_member_variable()。

#### 说明

一般来说，函数名的**每个单词首字母大写** (即 “驼峰变量名” 或 “帕斯卡变量名”)，**没有下划线**。对于首字母缩写的单词，更倾向于将它们视作一个单词进行首字母大写（例如，写作 StartRpc（）而非 StartRPC（））。



## 注释约定

### 注释风格

**统一风格**，使用**//**或者**/* */**均可，但**//更常用**

### 类注释

每个类的定义都要附带一份注释， 描述类的功能和用法，除非它的功能相当明显。

### 函数注释

函数声明处的注释描述函数功能；定义处的注释描述函数实现。



## 头文件中的#define保护（用的不多）

- 目的：防止头文件被多次包含而造成编译错误

- 举例：

  ```c++
  #ifndef __XXX_H__
  #define __XXX_H__
  
  int a=1;
  
  #endif
  ```

  其伪代码如下：

  ```
  如果(没有定义宏__XXX_H__)
  {
      那么直接定义宏__XXX_H__
      定义变量a，并且赋值为1
  }
  结束程序
  ```

  总结：无论头文件被包含多少次，变量a只被定义一次，不会有重复包含重复定义的问题存在

- ==<u>**现常用此方法：#pragma once**</u>== 



## 预处理include

- 基于google规范的**顺序**： 相关头文件，C 库， C++ 库， 其他库的.h， 本项目内的.h

- ROS中C++常用的include

  - `import rospy = #include "ros/ros.h"`

  - `from pix_driver_msgs.msg import GearCommand = #include "pix_driver_msgs/GearCommand.h"`

  - `from autoware_msgs.msg import VehicleCmd = #include ''autoware_msgs/VehicleCmd.h''`

  - `import math = #include<cmath>`

  - `import time  = #include<ctime>`

  - class ×× **：** = class ××  **{}**



## 基础数据类型

- 基本分类：

  - bool型
  - 整型 （char型从本质上说，也是种整型类型，它是长度为1的整数，通常用来存放字符的ASCII码）
  - 浮点型

- int_t同类

  是typedef定义的**表示标志**，是一种表示规范，是通过typedef给类型起的别名，而不是一种新的数据类型。

  举例：

  ```c++
  typedef signed char             int8_t;
  typedef short int               int16_t;
  typedef int                     int32_t;
  typedef long int                int64_t;
  
  typedef unsigned char           uint8_t;
  typedef unsigned short int      uint16_t;
  typedef unsigned int            uint32_t;
  typedef unsigned long int       uint64_t;
  ```

  - 注意：**uint8_t实际是一个char**，输出uin8_t类型的变量**实际输出**的是其**对应的字符**，而不是真实数字。




## ==指针！==

- 目的：用来存储变量在内存中的地址

  - 一般变量的存储地点：CPU中的**缓存**，即属于RAM中的SRAM（静态RAM），而内存条为DRAM（动态RAM）

  ![image-20231113215557022](../imgs/image-20231113215557022.png)

- 声明格式：变量类型   * 变量名（多用于C）   或     **变量类型 *  变量名（多用于C＋＋）**

  - **注意：声明处的‘*’仅仅是声明，不作为间接运算符** 

  - &：取地址运算符，**获取变量地址**

  - *：间接运算符/解除引用运算符，**获取某地址对应的值**

举例：

```c++
int a=5;//设a的地址为EE，b的地址为FF
int* b=&a;//将b指向a，赋值为a的地址（等于指针指向该变量）
此时：
&b=FF；
b=EE；
*b=5;

若此时有：*b=100；
则：a=100；
```

![image-20231113220254104](../imgs/image-20231113220254104.png)

![image-20231113220314286](../imgs/image-20231113220314286.png)

- 数组名：多数情况下，C++将数据名视为数组的**第1个元素的地址**

- 指针算术：C++允许将指针与整数相加，加1的结果等于**原来的地址值加上指向的对象占用的总字节数**



## 类和对象

一般形式为：

```c++
class 类名
{
  public:
    <公有数据和函数>
  private:
    <私有数据和函数>
  protected:
    <保护数据和函数>
}；
```



### 对象成员的访问

- **圆点访问**方式：
  - 对象名.成员名 
  - (*指向对象的指针).成员名  
- **指针访问**方式：
  - 对象指针变量名->成员名
  - (&对象名)->成员名  



### 构造函数

- 格式： 

  ```c++
  <类名>：：<类名>（<形参表>）
  {
      <函数体>
  }
  ```

- 特点：

  - 必须为**公有**
  - **函数名与类名相同**
  - 可以重载
  - 不仅指定返回类型
  - 不能被显式调用

- **无定义任何构造函数**时，系统会**自动生成一个无参数、空函数体的默认构造函数**



### class中的拷贝构造函数（构造函数后）

- 拷贝构造函数：

  如果类中没有定义拷贝构造函数，编译器将提供一个拷贝构造函数，功能是把**已存在对象的成员变量赋值给新对象的成员变量**。

  - **用一个已存在的对象创建新的对象**格式：

  ```c++
  类名 新对象名（已存在的对象名）;
  类名 新对象名 = 已存在的对象名;
  ```

  - **拷贝构造函数**格式：

  ```c++
  类名（const 类名& 对象名）{……}
  ```

  - 特点：

    - 必须为**公有**
    - 函数名必须**与类名相同**
    - 没有返回值  
    - 如果类中定义了拷贝构造函数，编译器将不提供拷贝构造函数
    - 拷贝构造函数可以重载，可以有默认参数

    ```c++
    类名（……,const 类名& 对象名,……）{……}
    ```



### 析构函数

-  格式：

  ```c++
  <类名>：：~<类名>（）
  {
    
  }
  ```

- 特点：
  - 必须为**公有**
  - 名字同类名，与构造函数名的区别在于**前面加~**，表明**功能相反**
  - 一个类中**只能定义一个析构函数**
  - 不能指定返回类型
  - 在释放一个对象时被自动调用
- **无定义任何析构函数**时，系统会**自动生成一个默认的构造函数**



### this指针

- 用途：
  - 为了区分成员和非成员，可通过：this-> 成员名 /（*this）.成员名
  - 类的方法需要返回当前对象的引用  
- 是一个由C＋＋编译器自动产生且较常用的一个隐含对象指针，不能被显式声明
- 是一个局部变量，局部于某个对象
- 是一个const指针，不能修改或给它赋值



### 类的静态成员

指声明为**static的成员**，在类的范围内**所有对象共享该数据**

- 引用格式：
  - <类名>：：<静态成员>
  - 对象名.共有静态成员
  - 对象指针->静态成员
  
- 静态数据成员：
  - 格式：<数据类型> <类名>：：<静态数据成员名>=<初始值> 
  
- 静态成员函数：
  - 定义：在一般函数**定义前加上static关键字** 
  
  - 不访问类中的非静态成员
  
    

### const

保护即需要共享、又需要防止改变的数据，应该声明为常量进行保护

- 常数据成员————const修饰数据成员
- 常成员函数————const修饰成员函数
  - 声明格式：类型 函数名（参数表） const； 
- 常对象————const修饰类的对象
  - 定义格式：const 类名 对象名； / 类名 const 对象名；
    - 如：const int foo



## 类型转换

尽量使用如**static_cast<>（）**这种显式类型转换，而不要使用 int y = (int)x 或 int y = int(x) 等转换方式



##ros::NodeHandle类

`ros::NodeHandle`是一个重要的类，它允许你与ROS系统进行通信。它提供了一个接口，用于**访问ROS的功能，例如创建发布者、订阅者、服务、参数等**。在C++中，可以通过创建 `ros::NodeHandle`对象来访问ROS的各种功能。

`ros::NodeHandle` 的对象通常在ROS节点的构造函数中创建，然后在整个节点的生命周期中被用于与ROS系统进行通信。通过使用 `ros::NodeHandle` 对象，你的程序可以方便地与其他ROS节点进行消息交换和服务调用。






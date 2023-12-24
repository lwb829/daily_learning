# Vehicle Dynamics and Control

## Chapter 1  Introduction（介绍）

### 1.2  ACTIVE STABILITY CONTROL SYSTEMS（主动稳定控制系统）

yaw stability control 偏航稳定性控制

roll over stability control 侧翻稳定性控制

**对于 Figure1-1中的摩擦系数u对比**：u越高，越能满足车辆过弯所需的侧向力，则越容易满足驾驶员的转向输入所遵循的轨迹；u越低，车辆越容易沿着半径更大的轨迹拐弯，特别地，当u特别低时，它不可能完全实现标称偏航率运动。



### 1.4  TECHNOLOGIES FOR ADDRESSING TRAFFIC CONGESTION（解决交通拥堵的技术）

#### 1.4.2 “Traffic-friendly” adaptive cruise control

- **ACC**：主要利用了雷达技术，通过毫米波雷达，发射毫米波段的电磁波，利用障碍物反射波的时间差确定障碍物距离，利用反射波的频率偏移确定相对速度。

- **原理：距离测量—>确定前车速度—>确定前车位置—>确定调节车辆**



#### 1.4.3 Narrow tilt-controlled commuter vehicles

**ITS**：intelligent transportation systems 智慧交通系统



### 1.5  EMISSIONS AND FUEL ECONOMY（排放与燃油经济性）

#### 1.5.1 Hybrid electric vehicles（HEV）（混合动力电动汽车）

- 优点：结合传统**内燃机**（ICE）和**电动机**，可以获得更大的续航里程，并减轻排放量，提高燃油经济性。

- 动力系统：
  - 串联：汽油**发动机带动发电机**，发电机可以为电池充电，也可以为驱动变速器的电动机提供动力
  - 并联：燃气发动机和电动机都独立连接到变速器上，二者均可提供推进动力



## Chapter 2  Lateral Vehicle Dynamics（车辆横向动力学）

### 2.1 LATERAL SYSTEMS UNDER COMMERCIAL DEVELOPMENT（商业发展中的横向系统）

- 解决车道偏离事故的三种类型的横向系统：

  - 车道偏离预警系统（**LDWS**）

    用一个由摄像头、车载电脑和软件组成，安装在挡风玻璃、仪表板或车顶上的AutoVue设备，来识别道路和车道标记之间的区别，计算机将这些数据与车辆速度结合，预测车辆何时开始漂移到意外的变道，当这种情况发生时发出声音提示，从而提醒驾驶员进行纠正。它在白天、夜晚的大多数环境条件下都可以有效运作。

  - 车道保持系统（**LKS**）

    自动控制方向盘，使车辆保持在车道内，并在转弯时跟随车道行驶。

    日产车推出的一种LKS，提供一种与驾驶员并行的自动转向。该系统只在“笔直”道路上运行(最终将指定最小半径)，并在规定的最低速度以上运行。前提是，在高速公路上连续行驶数小时后，司机会感到疲劳，因为他们不得不不断地轻微转向，以保持车辆在车道上。LKS试图通过提高直线公路的稳定性来减少这种疲劳。

    该系统使用单个CCD摄像头来识别车道划分，一个转向执行器来控制前轮，以及一个电子控制单元。

  - 偏航稳定控制系统 （**YSCS**）
  
    三种对于偏航控制的稳定控制系统：
  
    - Differential Braking Systems（差速制动系统）：利用车辆上的ABS制动系统，在左右轮之间施加差速制动来控制偏航力矩 （详细见8.2）
    - Steer-by-Wire Systems（线控转向系统）：修改驾驶员的转向角度输入，并为车轮增加一个校正转向角度 （详细见8.3）
    - Active Torque Distribution Systems（主动扭矩分配系统）：利用主动差速器和全轮驱动技术来独立控制分配给每个车轮的驱动扭矩，从而提供牵引力和偏航力矩的主动控制 （详细见8.4）



### 2.2 KINEMATIC MODEL OF LATERAL VEHICLE MOTION（车辆横向运动学模型）

车辆运动学模型把车辆完全视为刚体，主要考虑车辆的位姿（位置坐标、航向角），速度，前轮转角等关系，且不考虑任何影响运动的力的情况下，提供了车辆运动的数学描述。



#### 自行车运动学模型

- 四个假设：

  - 车辆在垂直方向的运动被忽略掉了，即我们描述的车辆是一个**二维平面上的运动物体**（可以等价与我们是站在天空中的俯视视角）
  - 假设车辆的结构就像自行车一样，即车辆的前两个轮胎拥有相同的的角速度和转速等，同样后面的两个轮胎也是如此，那么**前后的轮胎就可以各用一个轮胎来描述**
  - 假设车辆运动也和自行车一样，即**前轮胎控制车辆转角**
  - 当车速较低时，车辆前后轮的方向与车辆前后轮当前的速度方向相同



##### 基于后轴中心为原点的运动学模型

- 首先在一个二维平面上描述一个车辆：

  ![img](https://img-blog.csdn.net/20171202165342899?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvQWRhbVNoYW4=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

$\theta$为在**Y**$aw$方向的偏转角度，是相对于$x$轴的逆时针方向的角度，$v$是$\theta$方向的速度，$L$是车辆的轴距（前后轮胎的距离），$(x,y)$是车辆的坐标。

下面是该车辆的自行车模型：

![img](https://img-blog.csdn.net/20171202165432943?watermark/2/text/aHR0cDovL2Jsb2cuY3Nkbi5uZXQvQWRhbVNoYW4=/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70/gravity/SouthEast)

运动学自行车模型假定车辆行如一辆自行车，整个的控制量可以简化为$(a,\delta_f)$。其中$a$是车辆的**加速度**，踩油门即为正加速度，踩刹车即为负加速度；$\delta_f$是**方向盘转角**，即假定这个方向盘转角就是**前轮胎当前的转角**。



##### 基于车辆重心的运动学模型

![image-20231221192610754](C:\Users\李文博\AppData\Roaming\Typora\typora-user-images\image-20231221192610754.png)

我们假定车速较低，车辆路径半径R变化缓慢，那么车辆的方向变化率（$\dot{\psi}$）肯定等于车辆的角速度，所以车辆的角速度为$\dot{\psi}=\frac{v}{R}$。

在惯性坐标系**XY**下，可得到车辆运动学模型有三个输入：$\delta_f$，$\delta_r$，$V$

1. $\dot{X}=V\cos(\psi+\beta)$
2. $\dot{Y}=V\sin(\psi+\beta)$
3. $\dot{\psi}=\frac{V\cos{\beta}}{l_f+l_r}\left(\tan{\delta_f}-\tan{\delta_r}\right)$
4. $\beta=\arctan\left(\frac{l_{r}\tan(\delta_f)+l_{f}\tan(\delta_r)}{l_f+l_r}\right)$



##### 基于车辆重心的==前轮驱动==运动学模型（$\delta_r=0$）

![img](https://pic3.zhimg.com/v2-90c5d4da07dea2cf033f18f21bc0bfba_r.jpg)我们定义模型中的状态量，可以用四个状态量来描述车辆的当前状态：

  - $x$：车辆当前的$x$坐标

  - $y$：车辆当前的$y$坐标

  - $\psi$：车辆当前的偏航角（**Y**$aw$方向的偏角，往往用弧度来描述，逆时针方向为正）

  - $v$：车辆的速度

其中$l_f$和$l_r$为前轮和后轮到车辆重心C的距离，前后车轮与车辆纵向夹角为**滑移角$\delta_f$，$\delta_r$**。AO，BO两条直线分别垂直于两个滚动车轮的方向，点O定义为这两条直线的AO和BO的交点，也是车辆的瞬时滚动中心，该中心与重心C连线的长度**R为车辆路径的半径**。**速度$v$垂直于直线OC**，且相对于车辆纵向夹角为**滑移角$\beta$**。

==**各个状态量的更新公式如下：**==

1. $x_{t+1}=x_t+v_{t}\cos(\psi_{t}+\beta)dt$
2. $y_{t+1}=y_{t}+v_{t}\sin(\psi_{t}+\beta)dt$
3. $\psi_{t+1}=\psi_{t}+\frac{v_t}{l_r}\sin(\beta)dt$
4. $v_{t+1}=v_{t}+adt$
5. $\beta=\arctan\left(\frac{l_{r}}{l_f+l_r}\tan(\delta_f)\right)$ 

**注意**：

- 对于公式5：由于绝大多数的汽车后轮都不能够偏转，所以我们的自行车模型就假定后轮的转角控制输入$\delta_r$=0，即方向盘上的控制输入都反映到了前轮的转角上了。

- 对于公式3：$v_t{\sin{\beta}}$为t时刻的方向沿着垂直车辆中轴线的**线速度**，这部分方向的速度可以使其角度$\psi$改变，其中$v_t$是相对于重心而言的速度，故角度的变化仅需要除以$l_r$



#### 阿克曼转向几何模型

一种为了解决交通工具转弯时，内外转向轮路径指向的圆心不同的几何学。

实际上，前轮的左右转向角度并非完全相等，通常情况下，**内侧轮胎转角更大**。如下图所示，$\delta_o$和$\delta_i$分别为外侧前轮和内侧前轮偏角，当车辆右转时，右侧轮胎为内侧轮胎，其转角$\delta_i$较左前轮胎转角$\delta_o$更大。$l_w$为轮距，$L$为轴距，后轮两个轮胎转角始终为0$\degree$。

![image-20231222095229563](C:/Users/李文博/AppData/Roaming/Typora/typora-user-images/image-20231222095229563.png)

当滑移角$\beta$很小时，有公式表述为：$\frac{\dot{\psi}}{v} \approx \frac{1}{R}=\frac{\delta}{L} $ 或者$\delta=\frac{L}{R}$

由于内外侧轮胎的转向半径不同，因此有：$\delta_o=\frac{L}{R+\frac{l_w}{2}}$；$\delta_i=\frac{L}{R-\frac{l_w}{2}}$

**前轮平均转角**为：$\delta=\frac{\delta_o+\delta_i}{2}\cong \frac{L}{R}$

**内外转角之差**为：$\Delta{\delta}=\delta_i-\delta_o=\frac{L}{R^2}l_w=\delta^2 \frac{l_w}{L}$，因此前两个前轮的转向角的差异$\Delta{\delta}$与平均转向角的平方$\delta^2$成正比

![image-20231222103705370](C:/Users/李文博/AppData/Roaming/Typora/typora-user-images/image-20231222103705370.png)

上图所示，这种差动转向可以通过梯形拉杆装置获得。

依据阿克曼转向几何设计的车辆，沿着弯道转弯时，利用四连杆的相等曲柄使**内侧轮的转向角比外侧轮大约2~4度**，使四个轮子路径的圆心大致上交会于后轴的延长线上瞬时转向中心，让车辆可以顺畅的转弯。



### 2.3 BICYCLE MODEL OF LATERAL VEHICLE DYNAMICS（车辆横向动力学的自行车模型）

当**车速相对较高**时，则不能再假设车轮速度与车轮前轮方向一致，必须建立动力学模型。

通过对**轮胎和路面之间的复杂相互作用**来描述车辆的运动。在动力模型中，我们需要考虑的力大致分为两种：

- **纵向力**：使车辆前后移动的力

- **侧向力**：使车辆横向移动的力

**轮胎起到决定性作用**：纵向速度控制，通过控制轮胎转速实现；横向航向控制，通过控制轮胎转角实现。

**注意：当滑移角较大时，轮胎模型则不是线性的了。**



#### 受力分析

车辆上的作用力沿三个不同的轴分布：

- 纵轴上的力包括驱动力和制动力，以及滚动阻力和拖拽阻力作滚摆运动
- 横轴上的力包括转向力、离心力和侧风力，汽车绕横轴作俯仰运动
- 立轴上的力包括车辆上下振荡施加的力，汽车绕立轴作偏摆或转向运动



##### 车辆受力模型

![img](https://img-blog.csdnimg.cn/05ffb7985d20456dadf8d82cadf23905.png#pic_center)

在单车模型假设的前提下，再做如下假设则为简单的动力学模型：

- 只考虑纯侧偏轮胎特性，忽略轮胎力的纵横向耦合关系
- 用单车模型来描述车辆的运动，不考虑载荷的左右转移
- 忽略横纵向空气动力学



##### 车辆动力学模型

![img](https://img-blog.csdnimg.cn/eb49ca7075d345a7aa57d1d7ce28a2fe.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAMC0-b28=,size_20,color_FFFFFF,t_70,g_se,x_16)

**OXY**为固定于地面的惯性坐标系，**oxy**为固定于车身的车辆坐标系

车辆具有两个自由度（2自由度模型）：绕z轴的**横摆运动**；沿y轴的**横向运动**

根据牛顿第二定律，分别沿x，y，z轴做受力分析：

- x轴：$m\cdot a_x=F_{xf}+F_{xr}$

- y轴：$m\cdot a_y=m\cdot \left( \ddot{y}+V_x\dot{\psi}\right)=F_{yf}+F_{yr}$ （见下）

- ==z轴：$l_z\cdot \ddot{\psi}=l_f\cdot F_{yf}-l_r\cdot F_{yr}$ （力矩平衡）==

其中$m$为整车质量，$l_z$为车辆绕z轴转动的转动惯量



#### 横向动力学

![img](https://img-blog.csdnimg.cn/c8670cba8d3440f5a4fa562bdf1f2997.png#pic_center)

##### x、y轴方向公式

x、y轴方向加速度$a_x$，$a_y$由两部分构成：$x$、$y$轴方向的相关**位移加速度**$\ddot{x}$、$\ddot{y}$和**向心加速度**$V_y\dot{\psi}$、$V_x\dot{\psi}$ (线速度×角速度)，即公式为：==$a_y=\ddot{y}+V_x\dot{\psi}$；$a_x=\ddot{x}-V_y\dot{\psi}$==



##### 轮胎公式

由于轮胎受到的横向压力，轮胎会有一个很小的滑移角，如下图所示：

![img](https://img-blog.csdnimg.cn/4371979ec02f4a03b4971bf3c29ae35d.png)

- 前轮/后轮滑移角
  - 前轮：$\alpha_f=\delta-\theta_{Vf}$，其中$\delta$为前轮转角，$\theta_{Vf}$为前轮速度转角
  - 后轮：$\alpha_r=-\theta_{Vr}$，其中$\theta_{Vr}$为后轮速度转角

- 前轮/后轮横向轮胎力（与滑移角成正比）

  - 前轮：==$F_{yf}=2C_{\alpha f} \left( \delta-\theta_{Vf}\right)$==
  - 后轮：==$F_{yr}=2C_{\alpha r} \left( -\theta_{Vr}\right)$==

  **注意**：$C_{\alpha f}$、$C_{\alpha r}$分别为前后轮的**侧偏刚度(cornering stiffness)**，且由于车辆前后各有**两个轮子**，所以受力要**乘以2**。

- 前轮/后轮速度转角

  - 前轮：$\tan \left(\theta_{V_{f}}\right)=\frac{V_{y}+l_{f} \dot{\psi}}{V_{x}} \stackrel{\dot{y}=V_y}{\longrightarrow}\frac{\dot{y}+l_{f} \dot{\psi}}{V_{x}} $
  - 后轮：$\tan \left(\theta_{V_{r}}\right)=\frac{V_{y}-\ell_{r} \dot{\psi}}{V_{x}} \stackrel{\dot{y}=V_y}{\longrightarrow}\frac{\dot{y}-\ell_{r} \dot{\psi}}{V_{x}} $

  其中$l_f$为前悬长度，$l_r$为后悬长度；$l_{f} \dot{\psi}$、$l_{r} \dot{\psi}$为车辆绕质心旋转的线速度，即为横向速度。



##### 全局坐标系下公式

1. ==$\dot{X}=v_x \cos (\delta_f) - v_y \sin (\delta_f)$==
1. ==$\dot{Y}=v_x \sin (\delta_f) - v_y \cos (\delta_f)$==



#### 考虑路堤角度

若考虑路堤角度，则$y$轴受力分析应该改写为

$m\cdot \left( \ddot{y}+V_x\dot{\psi}\right)=F_{yf}+F_{yr}+F_{bank}$ 

其中$F_{bank}=mg\sin(\phi)$，$\phi$为路标约定的路堤角度，如下图所示

![image-20231224210046371](C:/Users/李文博/AppData/Roaming/Typora/typora-user-images/image-20231224210046371.png)



### 2.4 MOTION OF A PARTICLE RELATIVE TO A ROTATING FRAME（粒子相对于旋转框架的运动）








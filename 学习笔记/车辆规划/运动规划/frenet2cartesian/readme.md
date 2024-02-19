# frenet/cartesian坐标系

> 本文主要介绍frenet坐标系和cartesian坐标系之间的转换关系

## 理论推导

参考[Baidu Apollo代码](https://github.com/ApolloAuto/apollo/blob/master/modules/common/math/cartesian_frenet_conversion.cc)

参考知乎笔记: [https://zhuanlan.zhihu.com/p/514864431](https://zhuanlan.zhihu.com/p/514864431)

## cartesian -> frenet

已知：cartesian坐标系下 $\left(x_e, y_e\right), v_e, a_e, \theta_e, \kappa_e$
求解：frenet坐标系下的 $s^*, l^*, \theta r, \dot{s}, \ddot{s}, i, \ddot{l}, l^{\prime}, l^{\prime \prime}$

$$
\text { Frenet坐标系下位置状态 }\left\{\begin{aligned}
\hat{s} & =s_{\text {init }}-\frac{D^{\prime}\left(s_{\text {init }}\right)}{D^{\prime \prime}\left(s_{\text {init }}\right)}, D^{\prime}\left(s^*\right)=0, \text { 二阶最小化十牛顿法迭代 } \\
l^* & =\operatorname{sign}\left[\left(y_e-y\left(s^*\right)\right) \cos \theta_r-\left(x_e-x\left(s^*\right)\right) \sin \theta_r\right] 
\cdot \sqrt{\left[x_e-x\left(s^*\right)\right]^2+\left[y_e-y\left(s^*\right)\right]^2} \\
\theta_r & =\arctan \left[\frac{y^{\prime}\left(s^*\right)}{x^{\prime}\left(s^*\right)}\right]
\end{aligned}\right.
$$

$$
\text { Frenet坐标系下运动状态 }\left\{\begin{aligned}
\Delta \theta & =\theta_e-\theta_r \\
i & =v_e \sin (\Delta \theta) \\
l^{\prime} & =\left(1-\kappa_r l\right) \cdot \tan (\Delta \theta) \\
\dot{s} & =\frac{v_e \cos (\Delta \theta)}{\left(1-\kappa_r l\right)} \\
l^{\prime \prime} & =-\left(\kappa_r^{\prime} l+\kappa_r l^{\prime}\right) \cdot \tan (\Delta \theta)+\frac{\left(1-\kappa_r l\right)}{\cos ^2(\Delta \theta)} \cdot\left[\frac{\kappa_e\left(1-\kappa_r l\right)}{\cos (\Delta \theta)}-\kappa_r\right] \\
\ddot{s} & =\frac{1}{\left(1-\kappa_r l\right)} \cdot\left\{a_e \cdot \cos (\Delta \theta)-\dot{s}^2 \cdot\left[\left(\frac{\kappa_e\left(1-\kappa_r l\right)}{\cos (\Delta \theta)}-\kappa_r\right) l^{\prime}-\left(\kappa_r^{\prime} l+\kappa_r l^{\prime}\right)\right]\right\} \\
\ddot{l} & =a_e \cdot \sin (\Delta \theta)+v_e \cos (\Delta \theta)\left(v_e \kappa_e-\kappa_r \dot{s}\right)
\end{aligned}\right.
$$


## frenet -> cartesian

已知：frenet坐标系下的 $s^*, l^*, \theta r, \dot{s}, \ddot{s}, i, \ddot{l}, l^{\prime}, l^{\prime \prime}$
求解：cartesian坐标系下 $\left(x_e, y_e\right), v_e, a_e, \theta_e, \kappa_e$
$$
\text { 笛卡尔坐标系下位置状态 }\left\{\begin{array}{l}
\theta_r=\arctan \left[\frac{y^{\prime}\left(s^*\right)}{x^{\prime}\left(s^*\right)}\right] \\
\left(x_e, y_e\right) \equiv\left[x\left(s^*\right)+l^* \cdot \cos \left(\theta_r+\frac{\pi}{2}\right), y\left(s^*\right)+l^* \cdot \sin \left(\theta_r+\frac{\pi}{2}\right)\right] \\
\theta_e=\arctan \left(\frac{l^{\prime}}{1-\kappa_r l}\right)+\theta_r
\end{array}\right.
$$

$$
\text { 笛卡尔坐标系下运动状态 }\left\{\begin{aligned}
v_e & =\frac{\dot{s}\left(1-\kappa_r l\right)}{\cos (\Delta \theta)} \\
\kappa_e & =\left\{\frac{\left[l^{\prime \prime}+\left(\kappa_r^{\prime} l+\kappa_r l^{\prime}\right) \tan (\Delta \theta)\right] \cdot \cos ^2(\Delta \theta)}{1-\kappa_r l}+\kappa_r\right\} \cdot \frac{\cos (\Delta \theta)}{1-\kappa_r l} \\
a_e & =\frac{\ddot{s}\left(1-\kappa_r l\right)}{\cos (\Delta \theta)}+\frac{\dot{s}^2}{\cos (\Delta \theta)}\left\{l^{\prime}\left[\frac{\kappa_e\left(1-\kappa_r l\right)}{\cos (\Delta \theta)}-\kappa_r\right]-\left(\kappa_r^{\prime} l+\kappa_r l^{\prime}\right)\right\}
\end{aligned}\right.
$$

## 代码实现

**本目录下的src、include文件夹下的代码参考了apollo官方**，验证过可以正确完成坐标系间的转换

```bash
cd frene2cartesian
mkdir build
cd build
cmake ..
make
./frenet2cartesian
```

import numpy as np
import matplotlib.pyplot as plt
import math


def bezier_normal(Ps, n, t):
    """普通方式实现贝塞尔曲线

    Args:
        Ps (_type_): 控制点，格式为numpy数组：array([[x1,y1],[x2,y2],...,[xn,yn]])
        n (_type_): n个控制点，即Ps的第一维度
        t (_type_): 时刻t

    Returns:
        _type_: 当前t时刻的贝塞尔点
    """
    if n == 1:
        return Ps[0]
    p_t = np.array([0, 0])
    n = len(Ps)-1
    for i in range(n+1):
        C_n_i = math.factorial(n)/(math.factorial(i)*math.factorial(n-i))
        p_t = p_t+C_n_i*(1-t)**(n-i)*t**i*Ps[i]
    return p_t


# 画图验证
Ps = np.array([[0, 0], [1, 1], [2, 1], [3, 0], [3, 1]])
x_ = []
y_ = []
for t in np.arange(0, 1, 0.01):
    plt.cla()
    pos = bezier_normal(Ps, len(Ps), t)
    x_.append(pos[0])
    y_.append(pos[1])
    plt.plot(Ps[:, 0], Ps[:, 1])
    plt.scatter(x_, y_, c='r')
    # print(pos)
    # plt.plot(pos[0],pos[1])
    plt.pause(0.001)

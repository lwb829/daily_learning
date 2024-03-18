import numpy as np
import matplotlib.pyplot as plt
import copy
from celluloid import Camera  # 保存动图时用，pip install celluloid

# 递归的方式实现贝塞尔曲线


def bezier(Ps, n, t):
    """递归的方式实现贝塞尔曲线

    Args:
        Ps (_type_): 控制点，格式为numpy数组：array([[x1,y1],[x2,y2],...,[xn,yn]])
        n (_type_): n个控制点，即Ps的第一维度
        t (_type_): 步长t

    Returns:
        _type_: 当前t时刻的贝塞尔点
    """
    if n == 1:
        return Ps[0]
    return (1-t)*bezier(Ps[0:n-1], n-1, t)+t*bezier(Ps[1:n], n-1, t)


if __name__ == '__main__':
    d = 3.5  # 道路标准宽度

# 控制点
    Ps = np.array([
        [0, -d / 2],
        [25, -d / 2],
        [25, d / 2],
        [50, d / 2]
    ])

    n = len(Ps) - 1  # 贝塞尔曲线的阶数

    path = []  # 路径点存储
# 贝塞尔曲线生成
    for t in np.arange(0, 1.01, 0.01):
        p_t = bezier(Ps, len(Ps), t)
        path.append(p_t)
    path = np.array(path)

    # 画图
    fig = plt.figure(1)
    # plt.ylim(-4, 4)
    camera = Camera(fig)
    len_line = 50  # 车道长
    # 灰色路面图的四个顶点
    GreyZone = np.array([[- 5, - d - 0.5], [- 5, d + 0.5],
                        [len_line, d + 0.5], [len_line, - d - 0.5]])
    for i in range(len(path)):
        # plt.cla()
        # 画灰色路面
        plt.fill(GreyZone[:, 0], GreyZone[:, 1], 'gray')
        # 画白色分界线
        plt.plot(np.array([- 5, len_line]), np.array([0, 0]), 'w--')

        plt.plot(np.array([- 5, len_line]), np.array([d, d]), 'w')

        plt.plot(np.array([- 5, len_line]), np.array([- d, - d]), 'w')

        plt.plot(Ps[:, 0], Ps[:, 1], 'ro')  # 画控制点
        plt.plot(Ps[:, 0], Ps[:, 1], 'y')  # 画控制点连线
        # 设置坐标轴显示范围
        # plt.axis('equal')
        plt.gca().set_aspect('equal')
        # 绘制路径

        plt.plot(path[0:i, 0], path[0:i, 1], 'g')  # 路径点
        plt.pause(0.001)
    #     camera.snap()
    # animation = camera.animate()
    # animation.save('trajectory.gif')

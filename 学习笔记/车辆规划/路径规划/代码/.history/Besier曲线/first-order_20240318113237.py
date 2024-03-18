from celluloid import Camera  # 保存动图时用，pip install celluloid
import numpy as np
import matplotlib.pyplot as plt
P0 = np.array([0, 0])
P1 = np.array([1, 1])
fig = plt.figure(1)
camera = Camera(fig)
x = []
y = []
for t in np.arange(0, 1, 0.01):
    plt.plot([P0[0], P1[0]], [P0[1], P1[1]], 'r')
    p1_t = (1-t)*P0+t*P1
    x.append(p1_t[0])
    y.append(p1_t[1])
    # plt.plot(x,y,c='b')
    plt.scatter(x, y, c='b')
    # plt.pause(0.001)
    camera.snap()
animation = camera.animate()
animation.save('一阶贝塞尔.gif')

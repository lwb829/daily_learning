from celluloid import Camera  # 保存动图时用，pip install celluloid
import numpy as np
import matplotlib.pyplot as plt
P0 = np.array([0, 0])
P1 = np.array([1, 1])
P2 = np.array([2, 1])
fig = plt.figure(2)
camera = Camera(fig)

x_2 = []
y_2 = []
for t in np.arange(0, 1, 0.01):
    plt.cla()
    plt.plot([P0[0], P1[0]], [P0[1], P1[1]], 'k')
    plt.plot([P1[0], P2[0]], [P1[1], P2[1]], 'k')
    p11_t = (1-t)*P0+t*P1
    p12_t = (1-t)*P1+t*P2
    p2_t = (1-t)*p11_t+t*p12_t

    x_2.append(p2_t[0])
    y_2.append(p2_t[1])
    plt.scatter(x_2, y_2, c='r')
    plt.plot([p11_t[0], p12_t[0]], [p11_t[1], p12_t[1]], 'g')
    plt.title("t="+str(t))
    plt.pause(0.001)
#     camera.snap()
# animation = camera.animate()
# animation.save('2阶贝塞尔.gif')

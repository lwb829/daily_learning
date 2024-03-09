from celluloid import Camera
import numpy as np
import matplotlib.pyplot as plt
import math
import time

#相关车辆参数
L = 2  # 车辆轴距，单位：m
v = 2  # 初始速度
x_0 = 0  # 初始x
y_0 = -3  # 初始y
psi_0 = 0  # 初始航向角
dt = 0.1  # 时间间隔，单位：s
lam = 0.1  # 前视距离系数
c = 2  # 前视距离

class KinematicModel: #以后轮中心为车辆中心
    def __init__(self,x,y,psi,v,L,dt):
        self.x = x
        self.y = y
        self.psi = psi
        self.v = v
        self.dt = dt #离散模型
        self.L = L
        
    def update_state(self,a,delta_f):
        #假设控制量为转向角delta_f和加速度a
        self.x=self.x+self.v*math.cos(self.psi)*self.dt
        self.y=self.y+self.v*math.sin(self.psi)*self.dt
        self.psi=self.psi+self.v/self.L*math.tan(delta_f)*self.dt
        self.v=self.v+a*self.dt
        
    def get_state(self):
        return self.x,self.y,self.psi,self.v
    
def cal_target_index(robot_state,refer_path,l_d):
    #robot_state为车辆当前位置(x,y)
    #refer_path为参考轨迹（数组）
    dists=[]
    for xy in refer_path:
        dis = np.linalg.norm(robot_state-xy)  # 计算向量或矩阵的不同类型的范数
        dists.append(dis)
        
    min_index = np.argmin(dists)  # 找寻数组中最小元素的索引
    
    delta_l=np.linalg.norm(refer_path[min_index]-robot_state)
    #搜索前视目标点
    while l_d > delta_l and (min_index+1) < len(refer_path):
        delta_l = np.linalg.norm(refer_path[min_index+1]-robot_state)
        min_index += 1
    return min_index

def pure_pursuit_control(robot_state,current_ref_point,l_d,psi):
    #current_ref_point为参考路点
    #l_d为前视距离
    alpha = math.atan2(
        current_ref_point[1]-robot_state[1], current_ref_point[0]-robot_state[0])-psi  # 反正切值math.atan2(y/x)（弧度）
    delta=math.atan2(2*L*np.sin(alpha),l_d)
    return delta

def main():
    refer_path=np.zeros((1000,2))
    refer_path[:, 0] = np.linspace(0, 100, 1000)  # 直线
    refer_path[:, 1] = 2*np.sin(refer_path[:, 0]/3.0) + \
        2.5*np.cos(refer_path[:, 0]/2.0)  # 生成正弦轨迹

    ugv = KinematicModel(x_0, y_0, psi_0, v, L, dt)

    x_ = []
    y_ = []
    fig = plt.figure(1)
    # 保存动图用
    camera = Camera(fig)
    for i in range(600):
        robot_state = np.zeros(2)
        robot_state[0] = ugv.x
        robot_state[1] = ugv.y

        l_d = lam*ugv.v+c  # 注意，这里的运动学模型使用的速度v就是车身纵向速度vx
        ind = cal_target_index(robot_state, refer_path, l_d)  # 搜索前视路点

        delta = pure_pursuit_control(
            robot_state, refer_path[ind], l_d, ugv.psi)

        ugv.update_state(0, delta)  # 加速度设为0，恒速

        x_.append(ugv.x)
        y_.append(ugv.y)

        # 显示动图
        plt.cla()
        plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=1.0)
        plt.plot(x_, y_, "-r", label="trajectory")
        plt.plot(refer_path[ind, 0], refer_path[ind, 1], "go", label="target")
        # plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)

    plt.figure(2)
    plt.plot(refer_path[:, 0], refer_path[:, 1], '-.b', linewidth=1.0)
    plt.plot(x_, y_, 'r')
    plt.show()


if __name__ == '__main__':
    main()

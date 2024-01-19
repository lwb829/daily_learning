from celluloid import Camera
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math

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
        self.psi=self.psi+self.v/self.L*
"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import random
import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution #网格分辨率,即每一个小网格的边长
        self.rr = rr #机器人半径
        self.min_x, self.min_y = 0, 0 #坐标值
        self.max_x, self.max_y = 0, 0 #坐标值
        self.obstacle_map = None #障碍物地图
        self.x_width, self.y_width = 0, 0 #初始化网格地图的宽度和高度
        self.motion = self.get_motion_model() #初始化运动模型
        self.calc_obstacle_map(ox, oy) #计算障碍物地图,ox、oy为障碍物坐标列表

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # 节点x索引值
            self.y = y  # 节点y索引值
            self.cost = cost #从起始节点到当前节点的路径代价
            self.parent_index = parent_index #父节点在节点列表中的索引值

        def __str__(self): #定义节点的字符串表示形式（x,y,cost,parent_index）
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m] 起始点横坐标
            s_y: start y position [m] 起始点纵坐标
            gx: goal x position [m] 目标点横坐标
            gy: goal y position [m] 目标点纵坐标

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        # 计算起始节点和目标节点的索引值;cost=0;父节点=-1表示没有父节点
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        # 利用字典创建openlist和closelist集合,把起始节点'start_node'的索引放入openlist中
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            # 找到具有最小总代价的节点的ID，并将该节点赋值给current
            c_id = min(open_set,key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc") #蓝色交叉标记
                
                # 按下esc时退出程序
                plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # 检查当前节点是否与目标节点位置相同
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # 从openlist中删除c_id节点
            del open_set[c_id]

            # 将c_id节点放入closelist中
            closed_set[c_id] = current

            # 遍历每一种运动方式，获取周围的节点
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    # 通过每个节点的父节点，从目标节点反向追溯到起始节点
    def calc_final_path(self, goal_node, closed_set):
        # 将目标节点的实际坐标值加入到列表中
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    # 启发函数
    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # 启发函数权重
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    # 根据网格地图给定的索引值,转换为坐标系对应的位置坐标,与'calc_xy_index'函数操作相反
    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos
  
    # 将给定的实际位置坐标(x,y)转换为网格地图中的索引值:计算位置坐标和最小位置坐标之间的差值,在除以网格地图的分辨率,在对结果进行四舍五入,得到整数索引值
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    # 根据节点对象的x和y属性计算在网格地图中的索引:确定节点在y方向上的偏移量,乘以地图宽度;在加上节点在x方向上的偏移量,从而得到索引值
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    # 验证节点是否在有效的网格地图范围内
    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        
        # 确定地图大小
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        #初始化障碍物地图
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)] #遍历每个元素并初始化为False，表示没有设置障碍物
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y) #计算每一个障碍物位置与当前网格位置的欧式距离
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True #若距离小于机器人半径，则标记为障碍物
                        break
    
    #静态方法，创建一个运动模型列表，定义机器人可以采取的移动方式，包含增量(dx,dy)和移动代价cost 
    @staticmethod 
    def get_motion_model():
        # 横竖向cost=1，斜向cost=根号2
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 0.0  # [m]
    sy = 0.0  # [m]
    gx = 100 # [m]
    gy = 100 # [m]
    grid_size = 1.0  # [m]
    robot_radius = 0.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(0, 100):
        ox.append(i)
        oy.append(0.0)
    for i in range(0, 100):
        ox.append(0.0)
        oy.append(i)
    for i in range(0, 100):
        ox.append(i)
        oy.append(100.0)
    for i in range(0, 100):
        ox.append(100.0)
        oy.append(i)
    for i in range(-10, 23):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 8):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.1)
        plt.show()



if __name__ == '__main__':
    main()

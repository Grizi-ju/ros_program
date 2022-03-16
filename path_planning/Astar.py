"""
A* grid planning has not been improved
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
Modified by: Grizi-ju
Watch the video at bilibili, ID:小巨同学zz
Any questions, welcome the exchanges
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math
import numpy as np
import matplotlib.pyplot as plt

show_animation = True

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):

        self.resolution = resolution   # grid resolution [m]
        self.rr = rr       # robot radius [m]
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()


    # 创建一个节点类，节点的信息包括：xy坐标，cost代价,parent_index
    class Node:
        def __init__(self, x, y , cost, parent_index):
            self.x = x    # index of grid
            self.y = y
            self.cost = cost  # g(n)
            self.parent_index = parent_index   # index of previous Node

        def __str__(self):    
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)
 

    # 经过planning函数处理，传入sx,sy,gx,gy, 返回pathx,pathy(最终的路径)
    def planning(self, sx, sy, gx, gy):
        """
        1、sx = nstart  sy = ngoal
        2、open_set  closed_set
        3、open_set = nstart 
        4、将open表中代价最小的子节点=当前节点，并在plot上动态显示，按esc退出
        5、如果当前节点等于ngoal，提示找到目标点
        6、删除open表中的内容，添加到closed表中
        7、基于运动模型定义搜索方式
        8、pathx,pathy = 最终路径(传入ngoal,closed_set)，返回pathx,pathy
        """

        # 1、sx = nstart  sy = ngoal  初始化nstart、ngoal坐标作为一个节点，传入节点全部信息
        nstart = self.Node(self.calc_xyindex(sx, self.minx),  # position min_pos   2 (2.5)
                           self.calc_xyindex(sy, self.miny),  # 2 (2.5)
                           0.0,
                           -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                           self.calc_xyindex(gy, self.miny),
                           0.0,
                           -1)
        # 2、open表、closed表设定为字典
        # 3、起点加入open表 
        open_set, closed_set = dict(), dict()   # key - value: hash表
        open_set[self.calc_grid_index(nstart)] = nstart   

        while 1:
            if len(open_set) == 0:
                print("Open_set is empty...")
                break

        # 4、将open表中代价最小的子节点 = 当前节点，并在plot上动态显示，按esc退出  
          
            # f(n)=g(n)+h(n)  实际代价+预估代价  
            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(ngoal, open_set[o]))
            current = open_set[c_id]  

            # 将当前节点显示出来
            if show_animation:
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny),
                         "xc")   # 青色x 搜索点
                # 按esc退出
                plt.gcf().canvas.mpl_connect('key_release_event',
                                              lambda event: [exit(0) if event.key == 'escape' else None]
                                            )
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal!")
                ngoal.parent_index = current.parent_index
                print("ngoal_parent_index:",ngoal.parent_index)
                ngoal.cost = current.cost
                print("ngoal_cost:",ngoal.cost)
                break

            # 删除open表中的c_id的子节点,并把current添加到closed_set
            del open_set[c_id]
            closed_set[c_id] = current

            # 基于motion model做栅格扩展，也就是搜索方式，可进行改进，如使用双向搜索、JPS等
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,    # 当前x+motion列表中第0个元素dx
                                 current.y + move_y,
                                 current.cost + move_cost,c_id)
                n_id = self.calc_grid_index(node)   # 返回该节点位置index

                # 如果节点不可通过，跳过
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node   # 直接加入a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node    # This path is the best until now. record it

        pathx, pathy = self.calc_final_path(ngoal, closed_set) 
        
        return pathx, pathy

    
    def calc_final_path(self, ngoal, closedset):    # 传入目标点和closed表，经过函数处理得到最终所有的xy列表
        pathx, pathy = [self.calc_grid_position(ngoal.x, self.minx)], [
                        self.calc_grid_position(ngoal.y, self.miny)]
        parent_index = ngoal.parent_index
        while parent_index != -1:
            n = closedset[parent_index]
            pathx.append(self.calc_grid_position(n.x, self.minx))
            pathy.append(self.calc_grid_position(n.y, self.miny))
            parent_index = n.parent_index

        return pathx, pathy


                

    @staticmethod  # 静态方法，calc_heuristic函数不用传入self，因为要经常修改启发函数，目的是为了提高阅读性
    def calc_heuristic(n1, n2):  # n1: ngoal，n2: open_set[o]     
        h =  math.hypot(n1.x - n2.x, n1.y - n2.y)
        return h


    # 得到全局地图中的具体坐标: 传入地图中最小处障碍物的pos和index
    def calc_grid_position(self, index, minpos):   
        pos = index * self.resolution + minpos
        return pos

    # 位置转化为以栅格大小为单位的索引: 传入position,min_pos
    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.resolution)  # (当前节点-最小处的坐标)/分辨率=pos_index  round四舍五入向下取整

    # 计算栅格地图节点的index： 传入某个节点
    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)   

    # 验证是否为可通行节点
    def verify_node(self, node):
        posx = self.calc_grid_position(node.x, self.minx)
        posy = self.calc_grid_position(node.y, self.miny)

        if posx < self.minx:
            return False
        elif posy < self.miny:
            return False
        elif posx >= self.maxx:
            return False
        elif posy >= self.maxy:
            return False

        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.minx = round(min(ox))    # 地图中的临界值 -10
        self.miny = round(min(oy))    # -10
        self.maxx = round(max(ox))    # 60 
        self.maxy = round(max(oy))    # 60
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = round((self.maxx - self.minx) / self.resolution)   # 35
        self.ywidth = round((self.maxy - self.miny) / self.resolution)   # 35
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        self.obmap = [[False for i in range(int(self.ywidth))]
                       for i in range(int(self.xwidth))]
        for ix in range(int(self.xwidth)):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(int(self.ywidth)):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):  #将ox,oy打包成元组，返回列表，并遍历
                    d = math.hypot(iox - x, ioy - y)  
                    if d <= self.rr:          #代价小于车辆半径，可正常通过，不会穿越障碍物
                        self.obmap[ix][iy] = True
                        break



    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [
                    [1, 0, 1],
                    [0, 1, 1],
                    [-1, 0, 1],
                    [0, -1, 1],
                    [1, 1, math.sqrt(2)],
                    [1, -1, math.sqrt(2)],
                    [-1, 1, math.sqrt(2)],
                    [-1, -1, math.sqrt(2)]    
                 ]
        
        return motion




def main():
    print(__file__ + '  start!')
    plt.title("Astar")
    
    # start and goal position  [m]
    sx = -5.0
    sy = -5.0
    gx = 50
    gy = 50
    grid_size = 2.0
    robot_radius = 1.0

    # obstacle positions
    ox, oy = [],[]
    for i in range(-10, 60): 
        ox.append(i)
        oy.append(-10)      # y坐标-10的一行-10~60的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 60):
        ox.append(i)
        oy.append(60)       # y坐标60的一行-10~60的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)        # x坐标-10的一列-10~61的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 61):
        ox.append(60)
        oy.append(i)        # x坐标60的一列-10~61的坐标添加到列表并显示为黑色障碍物
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)        # x坐标20的一列-10~40的坐标添加到列表并显示为黑色障碍物
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)   # x坐标40的一列20~60的坐标添加到列表并显示为黑色障碍物



    if show_animation:
        plt.plot(ox,oy,".k") # 黑色.       障碍物
        plt.plot(sx,sy,"og") # 绿色圆圈    开始坐标
        plt.plot(gx,gy,"xb") # 蓝色x       目标点
        plt.grid(True)
        plt.axis('equal')  # 保持栅格的横纵坐标刻度一致


    a_star = AStarPlanner(ox, oy, grid_size ,robot_radius)  # grid_size=resolution 初始化中传入的参数
    pathx, pathy = a_star.planning(sx, sy, gx, gy)  # 开始与结束的坐标传入函数进行处理后，得到pathx,pathy：最终规划出的路径坐标


    if show_animation:
        plt.plot(pathx, pathy, "-r")  # 红色直线 最终路径
        plt.show()
        plt.pause(0.001)   # 动态显示


if __name__ == '__main__':
    main()

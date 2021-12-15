import math
import random
import matplotlib.pyplot as plt
import csv
import numpy as np

show_animation = True


class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []  # to define the path from parent to current---
            self.path_y = []  # the purpose is to check collision of the path
            self.parent = None

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=4, path_resolution=0.1, goal_sample_rate=3, max_iter=1000):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    self.node_list.append(final_node)
                    return self.generate_final_course(len(self.node_list) - 1)

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        '''extend the node list'''
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (x1, y1, leng) in self.obstacle_list:
        #     self.draw_obstacle(x1, y1,leng)
        x1,y1,leng=self.obstacle_list
        self.draw_obstacle(x1, y1, leng)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        # plt.axis("equal")
        # plt.axis([397360, 397400,3417985, 3418000])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    # def plot_circle(x, y, size, color="-b"):  # pragma: no cover
    #     deg = list(range(0, 360, 5))
    #     deg.append(0)
    #     xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    #     yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    #     plt.plot(xl, yl, color)
    def draw_obstacle(x1,y1,leng):
        plt.plot(x1, y1, "o",linewidth=2, label="number")

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        # dlist = [(node.x - rnd_node.x)  + (node.y - rnd_node.y)
        #           for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacleList):
        # x=list()
        # y=list()
        # leng=0
        x,y,leng=obstacleList
        if node is None:
            return False
        size=1.5

        for i in range(0,leng):
            x1=x[i]
            y1=y[i]
            # for (x1, y1) in obstacleList:
            dx_list = [x1 - x for x in node.path_x]
            dy_list = [y1 - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]
            #
            if min(d_list) <= size**1:
                return False  # collision
        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

def draw_vehicle(exampleFile,number):
    exampleReader_vehicle = csv.reader(exampleFile)
    exampleData_vehicle = list(exampleReader_vehicle)
    length_zu_vehicle = len(exampleData_vehicle)
    length_yuan_vehicle = len(exampleData_vehicle[0])

    x1 = list()
    y1 = list()
    # c1 = list()
    # num1=0

    for k in range(0, length_zu_vehicle):
        x1.append(float(exampleData_vehicle[k][0]))
        # c1.append(k)
        # num1=k
        y1.append(float(exampleData_vehicle[k][1]))

    # plt.plot(x1,y1, linewidth=2,label=number)
    plt.plot(x1,y1,"o")
    position=plt.ginput(2)
    print(position)
    return x1,y1,length_zu_vehicle,position

def show_path(rrt,path,show_animation):
    if show_animation:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        # plt.show()

def main(address,lines_sum):
    exampleFile_vehicle2 = open(address)  # 打开csv文件
    x_2, y_2, leng2, pos= draw_vehicle(exampleFile_vehicle2, 2)

    start_x=pos[0][0]
    start_y=pos[0][1]
    print("start:",start_x,",",start_y)

    gx=pos[1][0]
    gy=pos[1][1]

    x_2=x_2[::-1]
    y_2=y_2[::-1]
    print("end:",gx,",",gy)

    x1=x_2
    y1=y_2

    leng=leng2

    print(leng)
    obstacleList=(x1,y1,leng)
    print(x1,"\n",y1)

    x1_min=np.min(x1)
    x1_max=np.max(x1)
    y1_min=np.min(y1)
    y1_max=np.max(y1)
    if x1_min<y1_min:
        rand_area_min=x1_min
    else:
        rand_area_min=y1_min

    if x1_max<y1_max:
        rand_area_max=y1_max
    else:
        rand_area_max=x1_max

    print("random area min:",rand_area_min,"\n","random area max:",rand_area_max)
    eposide=0

    times=1
    lines_sum=lines_sum

    while eposide<lines_sum:
        print("第",times,"次\n")
        rrt = RRT(start=[start_x,start_y],
                  goal=[gx, gy],
                  # rand_area=[stat_x-1300, stat_y-1600],
                  rand_area=[rand_area_min, rand_area_max],
                  obstacle_list=obstacleList)
        path = rrt.planning(animation=show_animation)

        if path is None:
            print("Cannot find path")

        else:
            print("found path!!")
            eposide=eposide+1
            show_path(rrt,path,show_animation)
            save_path = path
            print("路径坐标：", save_path)
            f = open("./path/"+str(start_x)+'perception_path_'+str(eposide)+'.csv', 'w', encoding='utf-8', newline="")
            csv_writer = csv.writer(f)
            patt_point=len(path)
            for i in range(0,patt_point):
                csv_writer.writerow(save_path[i])

        times=times+1

if __name__ == '__main__':
    address='./boundary_combine/bound_combine.csv'
    lines_sum=1
    main(address,lines_sum)
import numpy as np
from matplotlib import pyplot as plt
from .Config import X_MAX, Y_MAX, Wall_X, safe_guard, platform_W, platform_H, ARRIVAL_Epsilon

def convert_X(x):
    return x + X_MAX

def convert_Y(y):
    return y + Y_MAX

def init_map(platform_W, platform_H):
    robot_map = np.zeros((2*X_MAX, 2*Y_MAX))
    # set the wal==l
    #robot_map[:convert_X(Wall_X), :]  = 1

    # set the platform
    Y_min = convert_Y(0 - safe_guard)
    Y_max = convert_Y(platform_H + safe_guard)
    X_min = convert_Y(-int(platform_W/2) - safe_guard)
    X_max = convert_Y(int(platform_W/2) + safe_guard)

    robot_map[X_min:X_max, Y_min:Y_max] = 1
    robot_map[:X_min, Y_min+10:Y_max-10] = 1

    return robot_map


class MapEnvironment(object):
    def __init__(self):
        self.map = init_map(platform_W, platform_H)
        self.platform_xmax = int(platform_W/2) + safe_guard + 0.5
        self.platform_xmin = -int(platform_W/2) - safe_guard - 0.5
        self.platform_ymax = platform_H + safe_guard + 0.5
        self.platform_ymin = 0 - safe_guard - 0.5

        self.xlimit = [0, np.shape(self.map)[0]-1]
        self.ylimit = [0, np.shape(self.map)[1]-1]
        
        self.epsilon = ARRIVAL_Epsilon

    def convert_real_to_map(self, pos):
        pos_map = np.zeros((2, 1))

        pos_map[0, 0] = pos[0, 0] + X_MAX
        pos_map[1, 0] = pos[1, 0] + Y_MAX

        return pos_map


    def goal_criterion(self, config, goal_config):
        """ Return True if config is close enough to goal

            @param config: a [2 x 1] numpy array of a state
            @param goal_config: a [2 x 1] numpy array of goal state
        """        
        return self.compute_distance(config, goal_config) < self.epsilon

    def compute_distance(self, start_config, end_config):
        """ A function which computes the distance between
            two configurations. 

            @param start_config: a [2 x 1] numpy array of current state
            @param end_config: a [2 x 1] numpy array of goal state
        """
        
        return np.linalg.norm(start_config-end_config)

    def state_validity_checker(self, config):
        """ Return True if all states are valid

            @param config: a [2 x n] numpy array of states
        """
        config_map = self.convert_real_to_map(config)
        config_x = config_map[0, 0]
        config_y = config_map[1, 0]

        if config_x < self.xlimit[0] or config_x > self.xlimit[1] or config_y < self.ylimit[0] or config_y > self.ylimit[1]:
            return False
        else:
            if self.map[int(round(config_x)), int(round(config_y))] == 1:
                return False

        return True

    def h(self, config, goal):
        """ Heuristic function for A*

            @param config: a [2 x 1] numpy array of state
        """
        dis = self.compute_distance(config, goal)

        return dis

    def init_visualizer(self):
        """ Initialize visualizer
        """

        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)

        # Plot img
        visit_map = 1 - np.copy(self.map) # black is obstacle, white is free space
        self.ax1_img = self.ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

    def visualize_plan(self, plan=None, tree=None, visited=None):
        """
            Visualize the final path
            @param plan: a final [2 x n] numpy array of states
        """
        visit_map = 1 - np.copy(self.map) # black is obstacle, white is free space

        self.ax1.cla()

        if visited is not None:
            visit_map[visited == 1] = 0.5
        self.ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

        if tree is not None:
            for idx in range(len(tree.vertices)):
                if idx == tree.GetRootID():
                    continue
                econfig = tree.vertices[idx]
                sconfig = tree.vertices[tree.edges[idx]]
                x = [sconfig[0], econfig[0]]
                y = [sconfig[1], econfig[1]]
                self.ax1.plot(y, x, 'r')

        if plan is not None:
            for i in range(np.shape(plan)[1] - 1):
                x = [plan[0,i], plan[0,i+1]]
                y = [plan[1,i], plan[1,i+1]]
                plt.plot(y, x, 'b', linewidth=3)
                #self.fig.canvas.draw()
                #plt.pause(.025) 
        self.fig.canvas.draw()
        plt.show()
        #self.fig.canvas.draw()
        #plt.pause(1e-10) 
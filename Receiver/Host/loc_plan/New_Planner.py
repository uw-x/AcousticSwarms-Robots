import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from Milestone import Milestone
from .Map import MapEnvironment, X_MAX, Y_MAX, convert_X, convert_Y
from .Config import ROBOT_NUM, Astar_Epsilon, SETOFF_POINT


class Node:
    def __init__(self, pos, father, g, h):
        self.pos = pos
        self.father = father
        self.g = g
        self.h = h
        self.f = g + h
    def update(self, g, father):
        self.father = father
        self.g = g
        self.f = self.g + self.h

def key_name(config):
    return str(config[0, 0])+'_'+str(config[1, 0])

def find_min_node(open_list):
    min_key = None
    min_value = 1000000

    for k in open_list.keys():
        node = open_list[k]
        if node.f < min_value:
            min_value = node.f
            min_key = k
    
    return min_key


class Path_Planner(object):    
    def __init__(self, robot_num):
        print("Init the global Planner.....")
        self.env = MapEnvironment()
        self.epsilon = Astar_Epsilon
        self.visited = np.zeros(self.env.map.shape)
        self.robot_num = robot_num
        self.out_angles = np.linspace(0, np.pi, self.robot_num)
        self.out_angles = self.out_angles[::-1]
        self.back_angles = np.zeros_like(self.out_angles)

    def down_sample(self, plan, step_size = 8):
        plan_size = plan.shape[1]
        downsample_num = plan_size//step_size + 1

        down_index = np.linspace(0, plan_size-1, downsample_num)
        down_index = np.fix(down_index).astype(int)

        milestones = plan[:, down_index]
        
        return milestones

    def convert_struct(self, plan):
        result_list = []
        for i in range(0, plan.shape[1]):
            temp_stone = Milestone(plan[0, i], plan[1, i])
            result_list.append(temp_stone)
        return result_list

    def correct_first_point(self, start_point):
        #print(start_point)
        #print(self.env.platform_xmax, self.env.platform_xmin, self.env.platform_ymin, self.env.platform_ymax)
        if start_point[0, 0] < self.env.platform_xmax and start_point[0, 0] > self.env.platform_xmin and start_point[1, 0] > self.env.platform_ymin and start_point[1,0] < self.env.platform_ymax:
            print("start point is invalid adjusting.......")
            deltax1 = abs(start_point[0, 0] - self.env.platform_xmax)
            deltax2 = abs(start_point[0, 0] - self.env.platform_xmin)
            deltay1 = abs(start_point[1, 0] - self.env.platform_ymin)
            deltay2 = abs(start_point[1, 0] - self.env.platform_ymax)
            reset_id = np.argmin([deltax1, deltax2, deltay1, deltay2])
            if reset_id == 0:
                start_point[0, 0] = self.env.platform_xmax + 1
            elif reset_id == 1:
                start_point[0, 0] = self.env.platform_xmin - 1 
            elif reset_id == 2:
                start_point[1, 0] = self.env.platform_ymin - 1
            else:
                start_point[1, 0] = self.env.platform_ymax + 1
            return start_point

        else:
            return start_point


    def return_all_milestones(self, back, robot_id, init_pos=None):
        if back == True:
            robot_index = 0
            if robot_id < 0:
                robot_index = 0
            elif robot_id >= self.robot_num:
                robot_index = self.robot_num - 1
            else:
                robot_index = robot_id

            goal = np.array([[-2], [-8]])
            init_pos = init_pos.reshape((-1, 1)) 
            init_pos = self.correct_first_point(init_pos)

            plan = self.Plan(init_pos, goal)
            enter_point = np.array([[-1], [-4]])
            plan = np.concatenate((plan, enter_point), axis = 1)
            milestones = self.convert_struct(plan)
            return milestones, self.back_angles[robot_index]
        else:
            start_point1 = np.array([[-3.5], [25]])
            start_point0 = SETOFF_POINT

            robot_index = 0
            if robot_id < 0:
                robot_index = 0
            elif robot_id >= self.robot_num:
                robot_index = self.robot_num - 1
            else:
                robot_index = robot_id
            
            rot_angle = np.pi/2 - self.out_angles[robot_index]
            #print(rot_angle)
            self.back_angles[robot_index] = (rot_angle + np.pi)

            if(rot_angle >= 0):
                return [], rot_angle
            else:
                print(robot_index)
                if robot_index == 0:
                    goal = start_point0 + 30*np.array([[np.cos(rot_angle)], [np.sin(rot_angle)]]) 
                elif robot_index == 1:
                    goal = start_point0 + 30*np.array([[np.cos(rot_angle)], [np.sin(rot_angle)]]) 
                elif robot_index == 2:
                    goal = start_point0 + 20*np.array([[np.cos(rot_angle)], [np.sin(rot_angle)]]) 
                print("goal", goal)
                plan = self.Plan(start_point1, goal)
                plan = np.concatenate((start_point0, plan), axis = 1)
                milestones = self.convert_struct(plan)
                return milestones, rot_angle

    def Plan(self, start_config, goal_config):        
        open_list = {}
        close_list = {}

        cost = 0
        start_config = start_config.astype("int")
        if not self.env.state_validity_checker(start_config.reshape((-1, 1))) or not self.env.state_validity_checker(goal_config.reshape((-1, 1))):
            raise ValueError('Start and Goal state must be within the map limits')
            exit(0)

        start_node = Node(start_config, None, 0, 0)
        open_list[key_name(start_config)] = start_node
        while True:
            min_K = find_min_node(open_list)
            #print(min_K)
            node = open_list.pop(min_K)
            close_list[min_K] = node

            if_end = False
            
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    if dx == 0 and dy == 0: continue
                    neighbor_pos = node.pos + np.array([[dx], [dy]])
                    if not self.env.state_validity_checker(neighbor_pos): continue

                    self.visited[round(neighbor_pos[0,0]), round(neighbor_pos[1,0])] = 1

                    if self.env.goal_criterion(neighbor_pos, goal_config): 
                        if_end = True
                        cost = node.g + np.sqrt(dx**2 + dy**2)
                        
                    neighbor_name = key_name(neighbor_pos) 
                    if neighbor_name in close_list: continue

                    if neighbor_name in open_list:
                        new_g = node.g + np.sqrt(dx**2 + dy**2)
                        if open_list[neighbor_name].g > new_g:
                            open_list[neighbor_name].update(new_g, min_K)
                    else:
                        new_g = node.g + np.sqrt(dx**2 + dy**2)
                        new_h = self.env.h(neighbor_pos, goal_config)*self.epsilon
                        neighbor_node = Node(neighbor_pos, min_K, new_g, new_h)
                        open_list[neighbor_name] = neighbor_node

            if if_end: 
                break
                    
        plan_reverse = []
        plan_reverse.append(goal_config)
        current_node = node
        #self.nodes = close_list

        while True:
            plan_reverse.append(current_node.pos)
            father_node = current_node.father
            if father_node is None:
                break
            current_node = close_list[father_node]
        
        state_count = len(close_list.keys())
        
        plan = []

        for i in range(len(plan_reverse) - 1, -1, -1):
            pos = plan_reverse[i]
            plan.append(pos)
            #self.visited[round(pos[0,0]), round(pos[1,0])] = 1

        #print("States Expanded: %d" % state_count)
        #print("Cost: %f" % cost)

        plan= np.concatenate(plan, axis=1) 
        down_plan = self.down_sample(plan)

        return down_plan


if __name__ == "__main__":  

    init_pos = np.array([[10], [0]])
    goal = np.array([[0], [-7]])

    Global_Plan = Path_Planner(7)

    milestones, angle, plan= Global_Plan.return_all_milestones(True, 2, init_pos)
    print(plan)
    # print("angle", angle)
    plan[0, :] += X_MAX
    plan[1, :] += Y_MAX

    Global_Plan.env.init_visualizer()
    Global_Plan.env.visualize_plan(plan=plan, tree=None, visited=None)
    plt.show()
    
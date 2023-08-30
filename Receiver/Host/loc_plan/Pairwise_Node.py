from .SMACOF_LIB import localize_pairwise_anchors
import numpy as np
from .LOC_2D import extract_distance_single, local_2D_by_dis, local_2D_by_dis_without_outlier
import matplotlib.pyplot as plt


def plot_circle(ax, xy, radius, edgecolor='k', facecolor='w', **kwargs):
    """Plot a circle."""
    circle = plt.Circle(
        xy,
        radius=radius,
        fill=True,
        edgecolor=edgecolor,
        facecolor=facecolor,
        **kwargs)
    ax.add_artist(circle)

def plot_measure(ax, xy, radius, edgecolor='k', facecolor='w', line_type = '-',**kwargs):
    """Plot a circle."""
    #print(radius)
    #print(xy)
    circle = plt.Circle(
        xy,
        radius=radius,
        fill=False,
        edgecolor=edgecolor,
        facecolor=facecolor,
        linestyle=line_type,
        **kwargs)
    ax.add_patch(circle)

def localize_triangle_anchors(Dis_matrix, achor_pos, init_pos):
    N_total = Dis_matrix.shape[0] 
    N_achor = achor_pos.shape[0]  
    N_robot = init_pos.shape[0]

    anchor_dis = Dis_matrix[ N_robot:N_total, 0:N_robot]


    pred_pos = np.zeros((N_robot, 2))
    # print(achor_pos)

    remove_pair = []
    for i in range(N_robot):
        dis = anchor_dis[:, i:i+1]
        #print(dis)
        init_pos0 = init_pos[i, :]
        #print("---------------")
        #print(dis)
        #print(init_pos0)

        pos, remove_id = local_2D_by_dis_without_outlier(achor_pos, dis, [init_pos0[0],init_pos0[1],0 ])
        #print(i, remove_id)


        pred_pos[i, :] = pos[:2, :].ravel()

        if remove_id > -1:
            # print("Remove: ", i, remove_id)
            new_dis = np.linalg.norm(pred_pos[i, :] - achor_pos[remove_id, :]) 
            remove_pair.append((i, remove_id, new_dis))
        #print(pos)
        
        
        
        fig = plt.figure()
        ax = fig.gca(
            aspect='equal',
            xlim=( -80, 80),
            ylim=( -80, 80)
        )
        colors = ['b', 'g', 'r', 'm', 'y', 'k', 'c'] 
        for j in range(N_achor):
            plot_circle(ax, (achor_pos[j, 0], achor_pos[j, 1]), radius=1, edgecolor=colors[j], facecolor=colors[j])
            plot_measure(ax, (achor_pos[j, 0], achor_pos[j, 1]), dis[j, 0], edgecolor=colors[j])
        if remove_id > -1:
            plot_measure(ax, (achor_pos[remove_id, 0], achor_pos[remove_id, 1]), remove_pair[-1][2], edgecolor=colors[remove_id], line_type='--')
        plt.title(str(i))
    # plt.show()
        
    return pred_pos, remove_pair

class Pairwise_Node(object):
    def __init__(self, Robot_num, anchor_num, do_SMACOF = False):
        self.out_robot = Robot_num - 1
        self.anchor_num = anchor_num
        self.achor_id = -1
        self.Total_NUM = self.out_robot + anchor_num
        self.do_SMACOF = do_SMACOF
        
        self.pair_matrix = np.zeros((self.out_robot + 1, self.out_robot + 1))
        self.achor_matrix = np.zeros((anchor_num, self.out_robot))
        
        self.pos_matrix = np.zeros((anchor_num, 2))
        self.weight_matrix = np.ones((self.Total_NUM, self.Total_NUM))

        self.coarse_pos_matrix = np.zeros((self.out_robot + 1, 2))

        self.pairwise_idx = 0
        self.achor_idx = 0
        self.last_achor_chirp = None

    def update_anchor_chirp(self, pos, sample, chirp_idx):
        #print(sample)
        self.achor_id = chirp_idx
        self.last_achor_chirp = pos.ravel()
        self.pos_matrix[self.achor_idx, :] = pos.ravel()
        sample = sample.mean(0)
        assert sample.shape[0] ==  self.out_robot + 1
        #print(sample, sample[chirp_idx])
        dis_vector = extract_distance_single(sample, sample[chirp_idx]) # 1*Robot_Num
        print("achor chirp_idx{}: ".format(chirp_idx), dis_vector)

        self.achor_matrix[self.achor_idx, :] = np.concatenate( (dis_vector[:chirp_idx],  dis_vector[chirp_idx+1:] ), axis = 0)

        self.achor_idx += 1

    def update_out_robot_chirp(self, init_pos, sample, chirp_idx):
        #print(sample)
        sample = sample.mean(0)
        

        assert sample.shape[0] ==  self.out_robot + 1
        self.coarse_pos_matrix[chirp_idx, :] = init_pos.ravel()
        #print(sample, sample[chirp_idx])
        dis_vector = extract_distance_single(sample, sample[chirp_idx]) # 1*Robot_Num

        print("out chirp_idx{}: ".format(chirp_idx), dis_vector)
        dis_vector[chirp_idx] = 0
        
        self.pair_matrix[chirp_idx, :] = dis_vector

        self.pairwise_idx += 1


    def prepare_dis_matrix(self):
        #print(self.Total_NUM)
        dis_matrix = np.zeros((self.Total_NUM, self.Total_NUM))
        self.coarse_pos_matrix = np.delete(self.coarse_pos_matrix, [self.achor_id], axis=0)
        self.pair_matrix = np.delete(self.pair_matrix, [self.achor_id], axis=0)
        self.pair_matrix = np.delete(self.pair_matrix, [self.achor_id], axis=1)

        self.pair_matrix = (self.pair_matrix + self.pair_matrix.T)/2

        dis_matrix[0:self.out_robot, 0:self.out_robot] = self.pair_matrix

        dis_matrix[self.out_robot:self.Total_NUM, 0:self.out_robot] = self.achor_matrix 
        dis_matrix[ 0:self.out_robot, self.out_robot:self.Total_NUM] = self.achor_matrix.T

        for i in range(0, self.anchor_num):
            for j in range(i, self.anchor_num):
                pointi = self.pos_matrix[i, :]
                pointj = self.pos_matrix[j, :]
                d_temp = np.linalg.norm(pointi - pointj)
                dis_matrix[self.out_robot + i, self.out_robot + j] = d_temp
                dis_matrix[self.out_robot + j,self.out_robot + i] = d_temp
            
        return dis_matrix

    def do_localize(self):
        coarse_pos = None
        #print(self.pairwise_idx, self.out_robot, self.achor_idx, self.anchor_num)
        if self.achor_idx < self.anchor_num or self.pairwise_idx < self.out_robot:
            print("Pair wise process not finish!!!!!")
            return None
        else:
            dis_matrix = self.prepare_dis_matrix()
            # print(dis_matrix)
            #print(self.coarse_pos_matrix)
            if self.do_SMACOF:
                coarse_pos, remove_pair = localize_triangle_anchors(Dis_matrix = dis_matrix, init_pos = self.coarse_pos_matrix, achor_pos = self.pos_matrix)
                print("coarse_pos: ")
                print(coarse_pos)
                # print(remove_pair)
                if len(remove_pair)>0:
                    for robot_i, anchor_i, new_dis in remove_pair:
                        # self.weight_matrix[robot_i, self.out_robot + anchor_i] = 0.1
                        # self.weight_matrix[self.out_robot + anchor_i, robot_i] = 0.1
                        print("replace: ", dis_matrix[self.out_robot + anchor_i, robot_i], dis_matrix[robot_i, self.out_robot + anchor_i] ," with ", new_dis)
                        dis_matrix[self.out_robot + anchor_i, robot_i] = new_dis
                        dis_matrix[robot_i, self.out_robot + anchor_i] = new_dis

                for i in range(self.out_robot-1):
                    for j in range(i + 1, self.out_robot):
                        coarse_range = np.linalg.norm(coarse_pos[i] - coarse_pos[j])
                        measure_range = dis_matrix[i, j]
                        print("Robot {} and {}: ".format(i, j), coarse_range, measure_range)
                        if abs(measure_range - coarse_range) > 20:
                            print("warning discard range between {} and {} from {:.2f} to {:.2f}".format(i, j, measure_range, coarse_range))
                            # self.weight_matrix[i, j] = 0.1
                            # self.weight_matrix[j, i] = 0.1
                            dis_matrix[i, j] = coarse_range
                            dis_matrix[j, i] = coarse_range
                #print(self.weight_matrix)
                # print(dis_matrix)
                print(dis_matrix)
                pos = localize_pairwise_anchors(Dis_matrix = dis_matrix, init_pos = coarse_pos, achor_pos = self.pos_matrix, W = self.weight_matrix )
            else:
                pos, _ = localize_triangle_anchors(Dis_matrix = dis_matrix, init_pos = self.coarse_pos_matrix, achor_pos = self.pos_matrix)

            #print('Coarse pos matrix', self.coarse_pos_matrix)
            #print('Anchor pos matrix', self.pos_matrix)
            #print('Res shape', pos.shape)
            # raise KeyError
            pos = pos[:self.out_robot, :]
            result_pos = np.zeros((self.out_robot+1 , 2))
            #print(self.achor_id ,"....")
            #print(pos.shape ,"....")
            
            # if self.achor_id > 0:
            result_pos[:self.achor_id] = pos[:self.achor_id]
            
            # print('Result pos', result_pos[self.achor_id])
            #print('Last anchor chirp', self.last_achor_chirp)
            result_pos[self.achor_id] = self.last_achor_chirp.reshape(2)
            
            # if self.achor_id < pos.shape[-1] - 1:
            result_pos[self.achor_id+1:] = pos[self.achor_id:]

            return result_pos, coarse_pos
            # return dis_matrix


if __name__ == "__main__":
    node = Pairwise_node(7, 7)
    for i in range(0, 7):
        sample = np.random.rand(7)*100
        node.update_anchor_chirp(np.array([i, i]),sample, 0)

    for i in range(1, 7):
        sample = np.random.rand(7)*100
        node.update_out_robot_chirp(sample, i)

    dis_matrix = node.do_localize()
    print(dis_matrix)
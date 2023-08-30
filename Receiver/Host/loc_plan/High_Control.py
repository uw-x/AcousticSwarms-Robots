import numpy as np
from .Local_Node import Local_Node
from .New_Planner import Path_Planner
from enum import Enum
from .Config import SETOFF_POINT, LOC_METHOD, BACK_RANGE
from .utils import wrap_to_pi
from .LOC_2D import extract_distance_single, local_2D_by_dis
from .SMACOF_LIB import localize_pairwise_anchors


class High_Status(Enum):
    IN_PLATFORM = 0
    OUT_NAV = 1
    OUT_FORWARD = 2
    BACK_FORWARD = 3
    PAIRWISE_LOC = 4 
    BACK_NAV = 5

class High_Level_Controller(object):    
    def __init__(self, Robot_id, global_planner: Path_Planner):
        self.Robot_id = Robot_id
        self.Current_status = High_Status.IN_PLATFORM
        self.loc_node = None
        self.forwad_angle = 0
        self.back_range = -1
        self.global_planner = global_planner

    def process_action(self, new_status, landmarks = None, init_pos = None):
        self.Current_status = new_status
        if (new_status == High_Status.OUT_NAV):
            init_pos = np.array([[SETOFF_POINT[0, 0]], [SETOFF_POINT[1, 0]], [90]])
            init_cov = np.array([
                [0.64, 0.0, 0.0], [0.0, 0.64, 0.0], [0.0, 0.0, 0.03]
            ])
            
            milestones, angle = self.global_planner.return_all_milestones(False, self.Robot_id)
            self.forwad_angle = angle
            if len(milestones) > 0:
                save_name = "Robot_" + str(self.Robot_id) + "_out"
                if landmarks is None:
                    raise ValueError('The input landmark is None')
                self.loc_node = Local_Node(init_pos, init_cov, landmarks, True, save_name, Method = LOC_METHOD)
                stone_array = np.array([m.point for m in milestones]) 
                self.save_debug_data("mile_stones", stone_array)
            else:
                self.Current_status = High_Status.OUT_FORWARD

            return milestones, np.rad2deg(angle)

        elif (new_status == High_Status.OUT_FORWARD):
            if self.loc_node is not None:
                self.loc_node.save_debug_data()
                self.loc_node = None


        elif (new_status == High_Status.PAIRWISE_LOC):
            if init_pos is None:
                raise ValueError('The input init_pos is None')
            init_angle = np.deg2rad(init_pos[2])
            
            if self.back_range < 0:
                print("Warining!!! no back_range is saved")
                return None, None
            else:
                print("init_angle: ", init_angle)
                print("init_r: ", self.back_range)
                coarse_X = SETOFF_POINT[0, 0] + self.back_range*np.cos(init_angle + np.pi)
                coarse_Y = SETOFF_POINT[1, 0] + self.back_range*np.sin(init_angle + np.pi)

                init_pos = np.array([coarse_X, coarse_Y, init_angle])

                return init_pos, None

        elif (new_status == High_Status.BACK_NAV):
            if landmarks is None:
                raise ValueError('The input landmark is None')
            if init_pos is None:
                raise ValueError('The input init_pos is None')

            init_pos = init_pos.reshape((-1, 1))
            '''
            ## this part is only for debugging experiment
            init_angle = np.deg2rad(init_pos[2, 0])
            if self.back_range < 0:
                print("Warining!!! no back_range is saved")
            else:
                print("init_angle: ", init_angle)
                print("init_r: ", self.back_range)
                coarse_X = SETOFF_POINT[0, 0] + self.back_range*np.cos(init_angle + np.pi)
                coarse_Y = SETOFF_POINT[1, 0] + self.back_range*np.sin(init_angle + np.pi)
                
                init_pos = np.array([[coarse_X], [coarse_Y], [init_angle]])

            print(init_pos)
            '''
            milestones, angle = self.global_planner.return_all_milestones(True, self.Robot_id, init_pos[:2,:])
            
            #init_pos = np.array([init_pos[0, 0], init_pos[1, 0], wrap_to_pi(self.forwad_angle)])
            
            init_cov = np.array([
                [0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]
            ])

            save_name = "Robot_" + str(self.Robot_id) + "_back"
            self.loc_node = Local_Node(init_pos, init_cov, landmarks, True, save_name, Method = LOC_METHOD)
            stone_array = np.array([m.point for m in milestones]) 
            self.save_debug_data("mile_stones", stone_array)
            
            return milestones, np.rad2deg(angle)

        elif (new_status == High_Status.IN_PLATFORM):
            if self.loc_node is not None:
                self.loc_node.save_debug_data()
                self.loc_node = None

        return None, None

    def monitor_back_range(self, sample_offsets, self_offset):
        if (self.Current_status == High_Status.BACK_FORWARD):
            dis = extract_distance_single(sample_offsets, self_offset)
            self.back_range = dis
            if dis < BACK_RANGE:
                return True
            else:
                return False
        else:
            raise ValueError('monitor_bank_range is called in Wrong states')

    def update_offset(self, sample_offsets, self_offset, old_pos, new_pos):
        if(self.Current_status == High_Status.OUT_NAV or self.Current_status == High_Status.BACK_NAV):
            pred_pos, velocity, angle_corrected = self.loc_node.update_offset(sample_offsets, self_offset, old_pos, new_pos)
            return pred_pos, velocity, angle_corrected
        else:
            print("Warining!!! call update_offset in wrong status")
            return None, None, None

    def back_init_loc_mulitple(self, Dis_matrix, init_pos, achor_pos):
        new_pos = localize_pairwise_anchors(Dis_matrix, init_pos, achor_pos)
        return new_pos 

    def back_init_loc_single(self, sample_offsets, self_offset, landmarks, init_pos):
        
        achor_num = sample_offsets.shape[0]
        assert(landmarks.shape[0] == achor_num)
        dis = np.zeros((achor_num, 1))
        
        for i in range(0, achor_num):
            dis[i] = extract_distance_single(self_offset, sample_offsets[i])

        p0 = [init_pos[0, 0], init_pos[1, 0], 0]
            
        pred_pos = local_2D_by_dis(landmarks, dis, p0)

        return pred_pos[:2 :]

    def save_debug_data(self, name, value):
        if self.loc_node is not None:
            self.loc_node.save_input(name, value)

    def update_status(self, status):
        if self.loc_node is not None:
            self.loc_node.update_status(status)
    
    def update_middlepoint(self, middle_pos):   
        if self.loc_node is not None:
            self.loc_node.update_milestone(middle_pos)
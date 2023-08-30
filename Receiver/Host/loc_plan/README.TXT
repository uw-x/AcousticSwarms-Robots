1. initialization:
(1) Local_Node:
localization_node = Local_Node(initial_mean, initial_cov, alphas, beta, landmarks, init_angle_available, 'PF')

initial_mean - shape (3, 1), init pos of robot (x, y, theta) - (cm, cm, rad)
initial_cov - shape (3, 3) convariance matrix of init position, can use the example np.diag([0.2**2, 0.2**2, 0.1*2])
alphas - motion noise parameters need to be fine tune in experiment. Initailly use:  alphas = np.array([0.05**2, 0.005**2, 0.3**2, 0.005**2])
beta - chirp ranging noise, Initailly use: beta = np.diag([range_err**2, range_err**2, range_err**2]), where range_err = 0.7 cm
landmarks - shape (3, 2) position of three achors
init_angle_available - True - init angle is known; False - init angle is not known


(2) Planner node:
plan_node = Back_Planner(platform_W, platform_H, init_pos)

platform_W - the width of platform
platform_H - the height of platform
init_pos - the start point of robot, (x, y, theta) 
(destination is defalted by (0, 0))


2. Update 
(1) when the chirp data come, Call:
mean, velocity, particles = localization_node.update_offset(self, sample_offsets, self_offfset, old_pos = None, new_pos = None):

Input:
    sample_offsets - the sample offset from three achors - shape (3, 1)
    self_offfset - the samples offsets from self chirp - single number
    old_pos - the previous est position of robot - shape (3, ) (x, y, theta)
    new_pos - the current est potition of robot - shape (3, ) (x, y, theta)
Output:
    mean - the corretced potition of robot - shape (3, ) - (x, y, theta)
    velocity -  shape (2, 1) est velocity (vx, vy)
    particles - for debugging just ignore

(2) when the correction actions come, Call:
    localization_node.update_milestone(pos)

Input:
    pos - shape (3, ) the est position of the robot before rotating


(3) when the landmarks arrived, Call:

next_milestone, next_next_milestone = plan_node.update_milestone()

Output:
    next_milestone - next milestone, (x,y)
    next_next_milestone - the mile stone after the next milestone. 
The destination is [0, 0], when you is arriving or have arrived at [0, 0], 
the next_milestone returns (0, 0) and next_next_milestone returns (0, 1) for robot go further into the platform

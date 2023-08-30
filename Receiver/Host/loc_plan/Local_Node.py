import numpy as np
import matplotlib.pyplot as plt

from .utils import minimized_angle, wrap_to_pi
#from ekf import ExtendedKalmanFilter
from .pf import ParticleFilter
from .LOC_2D import extract_distance, local_2D_by_dis, offset_to_dis, extract_distance_single, local_2D_by_dis_without_outlier
from .Config import USE_DEGREE, PARTICLE_NUM, SAVE_DEBUG, SAVE_NAME, VELOCITY_CLIP, PRINT_DEBUG, SPEAKER_DIS, RESET_THRESHOLD, RESET_APPLY, Status, UPDATE_ANGLE, LEAST_POINT, ALPHAS, BETAS
import time
import json
from sklearn.linear_model import LinearRegression
from Milestone import Milestone
np.random.seed(2)

def estimate_angle(X, Y):
    # print(X, Y)
    model = LinearRegression().fit(X, Y)
    r_sq = model.score(X, Y)
    angle_est = np.arctan(model.coef_[0, 0])
    u1 = [np.cos(angle_est), np.sin(angle_est)]
    u2 = [np.cos(angle_est + np.pi), np.sin(angle_est + np.pi)]
    v = [X[-1, 0] - X[0, 0], Y[-1, 0] - Y[0, 0]]
    if (np.inner(u1, v) >  np.inner(u2, v)):    
        return [angle_est, r_sq]
    else:
        return [angle_est + np.pi , r_sq]

class Field:
    def __init__(self, alphas, beta, landmarks):
        self.alphas = alphas
        self.beta = beta
        self.landmarks = landmarks
    '''
        Kalman Filter functions
    '''
    def G(self, x, u):
        """Compute the Jacobian of the dynamics with respect to the state."""
        prev_x, prev_y, prev_theta = x.ravel()
        rot1, trans, rot2 = u.ravel()

        G = np.zeros((3,3))
        G[0,0] = 1
        G[1,1] = 1
        G[2,2] = 1
        G[0,2] = -trans*np.sin(prev_theta + rot1)
        G[1,2] = trans*np.cos(prev_theta + rot1)

        return G

        # YOUR IMPLEMENTATION HERE

    def V(self, x, u):
        """Compute the Jacobian of the dynamics with respect to the control."""
        prev_x, prev_y, prev_theta = x.ravel()
        rot1, trans, rot2 = u.ravel()

        V = np.zeros((3,3))
        V[0,0] = -trans*np.sin(prev_theta + rot1)  
        V[0,1] = np.cos(prev_theta + rot1)
        V[1,0] = trans*np.cos(prev_theta + rot1)
        V[1,1] = np.sin(prev_theta + rot1)
        V[2,0] = 1
        V[2,2] = 1

        return V
        # YOUR IMPLEMENTATION HERE

    def H(self, x):
        """Compute the Jacobian of the observation with respect to the state."""
        prev_x, prev_y, prev_theta = x.ravel()
        mic1 = np.array([prev_x, prev_y])
    
        H = np.zeros((3,3))

        for i in range(0, self.landmarks.shape[0]):
            lxy = self.landmarks[i, :]
            a,b = lxy
            l1 = np.linalg.norm(mic1 - lxy)
            ## l1 
            coff = 0.5/l1
            H[i, 0] = coff*(2*prev_x - 2*a)
            H[i, 1] = coff*(2*prev_y - 2*b)
            H[i, 2] = 0

        return H

    '''
        Particle Filter functions
    '''

    def forward_multiple(self, x, u):
        '''
            x - Nx3
            u - Nx3 or Nx6
        '''
        ctrl_dim = u.shape[1]
        x_next = np.zeros_like(x)
        #print(x_next.shape)
        if ctrl_dim == 3:
            rot1, trans, rot2 = u[:, 0], u[:, 1], u[:, 2] 
            theta = x[:, 2] + rot1
            x_next[:, 0] = x[:, 0] + trans * np.cos(theta)
            x_next[:, 1] = x[:, 1] + trans * np.sin(theta)
            x_next[:, 2] = wrap_to_pi(theta + rot2)

        elif ctrl_dim == 6:
            rot1, trans1, rot2, rot3, trans2, rot4 = u[:, 0], u[:, 1], u[:, 2],  u[:, 3], u[:, 4], u[:, 5]
            theta = x[:, 2] + rot1

            delta_x1 =  trans1 * np.cos(theta)
            delta_y1 =  trans1 * np.sin(theta)
            theta = theta + rot2 + rot3
            delta_x2 =  trans2 * np.cos(theta)
            delta_y2 =  trans2 * np.sin(theta)      

            x_next[:, 0] = x[:, 0] + delta_x1  + delta_x2
            x_next[:, 1] = x[:, 1] + delta_y1 + delta_y2
            x_next[:, 2] = wrap_to_pi(theta + rot4) 

        return x_next

    def forward(self, x, u):
        """Compute next state, given current state and action.

        Implements the odometry motion model.

        x: [x, y, theta]
        u: [rot1, trans]
        """
        prev_x, prev_y, prev_theta = x.ravel()
        ctrl_dim = u.shape[0]
        x_next = np.zeros(x.size)

        if ctrl_dim == 3:
            rot1, trans, rot2 = u

            theta = prev_theta + rot1
            x_next[0] = prev_x + trans * np.cos(theta)
            x_next[1] = prev_y + trans * np.sin(theta)
            x_next[2] = minimized_angle(theta + rot2)

        elif ctrl_dim == 6:
            rot1, trans1, rot2, rot3, trans2, rot4 = u
            theta = prev_theta + rot1
            delta_x1 =  trans1 * np.cos(theta)
            delta_y1 =  trans1 * np.sin(theta)
            theta = theta + rot2 + rot3
            delta_x2 =  trans2 * np.cos(theta)
            delta_y2 =  trans2 * np.sin(theta)      

            x_next[0] = prev_x + delta_x1  + delta_x2
            x_next[1] = prev_y + delta_y1 + delta_y2
            x_next[2] = minimized_angle(theta + rot4)   

        return x_next.reshape((-1, 1))


    def observe_mulitple(self, pos):
        """Compute observation, given current state and landmark ID.
        pos - Nx3
        x: [x, y, theta]
        marker_id: int
        """
        #x, y, theta = pos.ravel()
        N_particles = pos.shape[0] 
        mic1 = pos[:, :2]
        theta =  pos[:, 2]
        #print(mic1[0:10])
        mic1[:, 0] += np.sin(theta)*SPEAKER_DIS
        mic1[:, 1] -= np.cos(theta)*SPEAKER_DIS
        #print(mic1[0:10])
        #raise KeyboardInterrupt
        z = np.zeros((N_particles,  self.landmarks.shape[0]))

        for i in range(0, self.landmarks.shape[0]):
            #print(mic1)
            #print(self.landmarks[i, :])
            #print(mic1 - self.landmarks[i, :])
            d1 = np.linalg.norm(mic1 - self.landmarks[i, :], axis = 1)
            z[:, i] = d1

        return np.array(z)#.reshape((-1, 1))

    def observe(self, pos):
        """Compute observation, given current state and landmark ID.

        x: [x, y, theta]
        marker_id: int
        """
        x, y, theta = pos.ravel()
        mic1 = np.array([x, y ])
        mic1[0] += np.sin(theta)*SPEAKER_DIS
        mic1[1] -= np.cos(theta)*SPEAKER_DIS
       
        z = []
        for i in range(0, self.landmarks.shape[0]):
            d1 = np.linalg.norm(mic1 - self.landmarks[i, :])
            z.append(d1)

        return np.array(z).reshape((-1, 1))

    def noise_from_motion(self, u, alphas):
        """Compute covariance matrix for noisy action.

        u: [rot1, trans, rot2]
        alphas: noise parameters for odometry motion model
        """
        ctrl_dim = u.shape[0]

        if ctrl_dim == 3:
            variances = np.zeros(3)
            variances[0] = alphas[0] * u[0]**2 
            variances[1] = alphas[2] * u[1]**2 + alphas[3] * u[0]**2
            variances[2] = alphas[0] * u[2]**2 

        if ctrl_dim == 6:
            variances = np.zeros(6)
            variances[0] = alphas[0] * u[0]**2
            variances[1] = alphas[2] * u[1]**2 + alphas[3] * u[0]**2
            variances[2] = alphas[0] * u[2]**2 
            variances[3] = alphas[0] * u[3]**2 
            variances[4] = alphas[2] * u[4]**2 + alphas[3] * u[3]**2
            variances[5] = alphas[0] * u[5]**2  

        return np.diag(variances)
    

    def get_motion_ob_single(self, u, alphas):
        ob = np.zeros((3, 1))
        rot1, trans, rot2 = u
        total_angle = rot1 + rot2

        Gyro_var = alphas[0]*(rot1)**2 + alphas[0]*(rot2)**2
        total_rot = np.random.normal(total_angle, np.sqrt(Gyro_var))

        rot1_var = alphas[0]*(rot1)**2 + alphas[1]*trans
        rot1_ob = np.random.normal(rot1, np.sqrt(rot1_var))

        trans_var = alphas[2]*(trans)**2 + alphas[3]*(rot1)**2
        trans_ob = np.random.normal(trans, np.sqrt(trans_var))

        ob[0] = rot1_ob
        ob[1] = trans_ob
        ob[2] = total_rot - rot1_ob

        return ob

    def get_motion_ob(self, u):
        alphas = self.alphas
        ctrl_dim = u.shape[0]

        if ctrl_dim == 3:
            return self.get_motion_ob_single(u, alphas)
        else:
            ob = np.zeros((6, 1))
            ob[0:3, :] = self.get_motion_ob_single(u[0:3], alphas)
            ob[3:6, :] = self.get_motion_ob_single(u[3:6], alphas)
            return ob
    

    def likelihood(self, innovation, beta):
        """Compute the likelihood of innovation, given covariance matrix beta.

        innovation: x - mean, column vector
        beta: noise parameters for landmark observation model
        """
        #print(innovation)
        norm = np.sqrt(np.linalg.det(2 * np.pi * beta))
        inv_beta = np.linalg.inv(beta)

        return np.exp(-0.5 * innovation.T.dot(inv_beta).dot(innovation)) / norm

    def sample_noisy_action(self, u, alphas=None):
        """Sample a noisy action, given a desired action and noise parameters.

        u: desired action
        alphas: noise parameters for odometry motion model (default: data alphas)
        """

        if alphas is None:
            alphas = self.alphas

        cov = self.noise_from_motion(u, alphas)
        return np.random.multivariate_normal(u.ravel(), cov) #.reshape((-1, 1))

    def sample_noisy_actions(self, u, N_particle, alphas=None):
        """Sample a noisy action, given a desired action and noise parameters.

        u: desired action
        alphas: noise parameters for odometry motion model (default: data alphas)
        """
        if alphas is None:
            alphas = self.alphas

        cov = self.noise_from_motion(u, alphas)
        return np.random.multivariate_normal(u.ravel(), cov, N_particle) #.reshape((-1, 1))

    def sample_noisy_observation(self, x, beta=None):
        """Sample a noisy observation given a current state, landmark ID, and noise
        parameters.

        x: current state
        marker_id: int
        beta: noise parameters for landmark observation model (default: data beta)
        """
        if beta is None:
            beta = self.beta

        z = self.observe(x)
        return np.random.multivariate_normal(z.ravel(), beta).reshape((-1, 1))
    
    def get_figure(self):
        return plt.figure(1)

class Local_Node:
    def __init__(self, init_pos, init_cov, landmarks, init_angle_available, save_name, Method = 'Simple'):
        self.env = Field(ALPHAS, BETAS, landmarks)
        self.alphas = ALPHAS
        self.beta = BETAS
        self.init_cov = init_cov
        self.save_name = save_name

        self.prev_pos = init_pos[0:2,:]
        self.prev_time = -1
        self.now_time = -1
        self.r_threshold = 0.1
        
        self.current_status = 1
        self.history_status = [1]
        self.history_pos = []
        
        self.temp_save = {}

        if SAVE_DEBUG:
            self.save_json = {}
            
            self.save_json["alphas"] = ALPHAS.tolist()
            self.save_json["beta"] = BETAS.tolist()
            
            self.save_json["init_pos"] = init_pos.tolist()
            self.save_json["init_cov"] = init_cov.tolist()

            self.save_json["landmarks"] = landmarks.tolist()
            self.save_json["data"] = []

        #if Method == 'EKF':
        #    print("EKF filter is loaded")
        #    self.filt = ExtendedKalmanFilter(init_pos, init_cov, alphas, beta)
        if Method == 'PF':
            print("PF filter is loaded")
            self.filt = ParticleFilter(init_pos, init_cov, PARTICLE_NUM, ALPHAS, BETAS, init_angle_available)
        else:
            print("Simple Trilateration is loaded")
            self.filt = None
        self.milestone = None
        self.check_rot = False
    
    def check_far_away(self, dis, pos):
        landmarks = self.env.landmarks
        dis_pred = np.zeros((3, ))
        for jj in range(0, 3):
            dis_pred[jj] = abs(np.linalg.norm(pos[0: 2].ravel() - landmarks[jj, :]) - dis[jj, 0])
        err = np.mean(dis_pred)
        if PRINT_DEBUG: print("dis err :", err)
        if err > RESET_THRESHOLD:
            return True
        else:
            return False

    def reset_PF(self, dis):
        #print("----------RESET !!!!!!----------")
        landmarks = self.env.landmarks
        landmarks += np.random.normal(loc = 0.0, scale = 0.01, size = landmarks.shape)
        pred_pos = local_2D_by_dis(landmarks, dis)
        if PRINT_DEBUG: print("reset to pos", pred_pos)
        self.filt = ParticleFilter(pred_pos, self.init_cov, PARTICLE_NUM, self.alphas, self.beta, False)
        
        self.prev_pos =  pred_pos[0:2,:]
        self.prev_time = -1
        self.now_time = -1
        self.milestone = None
        self.check_rot = False

        return pred_pos, self.filt.particles
    
    def save_input(self, name, value):
        if SAVE_DEBUG:
            if (type(value) == list):
                self.temp_save[name] = value
            elif (type(value) == int or type(value) == float):
                self.temp_save[name] = str(value)
            elif (type(value) == np.ndarray):
                self.temp_save[name] = value.tolist()
            elif (type(value) == Milestone):
                self.temp_save[name] = value.point()
            else:
                print("warning! " + name + " (" + str(type(value)) + ") type is not supported to be save")


    def update_offset(self, sample_offsets, self_offset, old_pos, new_pos, delta_t = 0):
        '''
            sample_offsets - the sample offset from three achors - shape (3, )
            self_offfset - the samples offsets from self chirp - single number
            old_pos - the previous est position of robot - (x, y, theta)
            new_pos - the current est potition of robot - (x, y, theta)
        '''
        self.now_time = time.time()

        achor_num = sample_offsets.shape[0]
        assert(self.env.landmarks.shape[0] == achor_num)
        
        
        if SAVE_DEBUG:
            self.temp_save["self_offset"] = str(self_offset)
            self.temp_save["sample_offsets"] = sample_offsets.ravel().tolist()
            self.temp_save["old_pos"] = old_pos.ravel().tolist()
            self.temp_save["new_pos"] = new_pos.ravel().tolist()

        if USE_DEGREE:
            old_pos[2] = np.deg2rad(old_pos[2])
            new_pos[2] = np.deg2rad(new_pos[2])

        dis = np.zeros((achor_num, 1))
        for i in range(0, achor_num):
           dis[i] = extract_distance_single(self_offset, sample_offsets[i])
        
        if PRINT_DEBUG: print("dis: ", dis.ravel())

        if self.check_rot:
            if SAVE_DEBUG: self.temp_save["milestone"] = self.milestone.ravel().tolist()
            
            move1 = self.milestone[0:2] - old_pos[0:2]
            move_r1 = np.linalg.norm(move1)
            
            if(move_r1 < self.r_threshold):
                move_angle1 = old_pos[2]
            else:
                move_angle1 = np.arctan2(move1[1], move1[0])

            rot1 = minimized_angle(move_angle1 - old_pos[2])
            rot2 = minimized_angle(self.milestone[2] - move_angle1)

            move2 = new_pos[0:2] - self.milestone[0:2]
            move_r2 = np.linalg.norm(move2)

            if(move_r2 < self.r_threshold):
                move_angle2 = self.milestone[2]
            else:
                move_angle2 = np.arctan2(move2[1], move2[0])

            rot3 = minimized_angle(move_angle2 - self.milestone[2])
            rot4 = minimized_angle(new_pos[2] - move_angle2)

            motion_est = np.array([rot1, move_r1,  rot2, rot3, move_r2, rot4])

        else:
            move = new_pos[0:2] -  old_pos[0:2]
            move_r = np.linalg.norm(move)
            if PRINT_DEBUG: print("move_r: ", move_r)
            if(move_r < self.r_threshold):
                #move_angle = old_pos[2]
                move_angle = old_pos[2]
            else:
                move_angle = np.arctan2(move[1], move[0])
            
            rot1 = minimized_angle(move_angle - old_pos[2])
            rot2 = minimized_angle(new_pos[2] - move_angle)

            motion_est = np.array([rot1, move_r, rot2])
        

        if PRINT_DEBUG: print("motion est:", motion_est)
        ## processing the data
        if self.filt is None:
            p0 = [self.prev_pos[0, 0], self.prev_pos[1, 0], 0]
            
            pred_pos, _ = local_2D_by_dis_without_outlier(self.env.landmarks, dis, p0)
            
            est_angle = [0, 0]
            if UPDATE_ANGLE:
                if self.current_status == 2:
                    #if 2 in self.history_status:
                    #    self.history_pos.clear()
                    self.history_pos.append(pred_pos[0:2].ravel())

                    if len(self.history_pos) > LEAST_POINT: 
                        print("trying to making angle......")
                        pos = np.array(self.history_pos)  
                        est_angle = estimate_angle(pos[-(LEAST_POINT+1):, 0:1], pos[-(LEAST_POINT+1):, 1:2])
                        self.temp_save["est_angle"] = est_angle
                        self.temp_save["pos0"] = pos[:, 0].ravel().tolist()
                        self.temp_save["pos1"] = pos[:, 1].ravel().tolist()
                else:
                    self.history_pos.clear()
                #if len(self.history_status) > 0:
                #    self.temp_save["history_status"]  = self.history_status
            self.history_status = []

            ##  calculayte the velocity 
            if self.prev_time == -1:
                velocity = np.zeros((2,1))
                self.prev_time = self.now_time
            else:
                delta_t = self.now_time - self.prev_time
                self.prev_time = self.now_time
                velocity = (pred_pos[:2] - self.prev_pos[:2])/delta_t 

            self.prev_pos = pred_pos
            self.check_rot = False
            
            velocity_abs = np.linalg.norm(velocity)
            if velocity_abs > VELOCITY_CLIP:
                velocity = velocity*VELOCITY_CLIP/velocity_abs
            
            if SAVE_DEBUG:
                self.temp_save["pred_pos"] = pred_pos.ravel().tolist()
                self.temp_save["delta_t"] = str(delta_t)
                self.temp_save["velocity"] = velocity.ravel().tolist()
                self.save_json["data"].append(self.temp_save)
                self.temp_save = {}

            return np.array([pred_pos[0, 0], pred_pos[1, 0], 0]), velocity, est_angle

        else:
            motion_est = motion_est.reshape((-1, 1))

            pred_pos, particles = self.filt.update(self.env, motion_est, dis)

            if self.prev_time == -1:
                velocity = np.zeros((2,1))
                self.prev_time = self.now_time
            else:
                if delta_t > 0:
                    #velocity = (pred_pos[0:2, :] - self.prev_pos)/delta_t 
                    if self.check_rot:
                        velocity = np.zeros((2,1))
                    else:
                        if delta_t > 1e-4:
                            velocity = (pred_pos[0:2, :] - self.prev_pos)/delta_t 
                        else:
                            velocity = np.zeros((2,1))
                else:
                    delta_t = self.now_time - self.prev_time
                    self.prev_time = self.now_time
                    if self.check_rot:
                        velocity = np.zeros((2,1))
                    else:
                        if delta_t > 1e-4:
                            velocity = (pred_pos[0:2, :] - self.prev_pos)/delta_t 
                        else:
                            velocity = np.zeros((2,1))
                    
            self.prev_pos = pred_pos[0:2, :] 
            self.check_rot = False
            velocity_abs = np.linalg.norm(velocity)
            if velocity_abs > VELOCITY_CLIP:
                velocity = velocity*VELOCITY_CLIP/velocity_abs

            if SAVE_DEBUG:
                self.temp_save["pred_pos"] = pred_pos.ravel().tolist()
                self.temp_save["delta_t"] = str(delta_t)
                self.temp_save["velocity"] = velocity.ravel().tolist()
                self.save_json["data"].append(self.temp_save)
                self.temp_save = {}

            return pred_pos, velocity, particles

    def update_milestone(self, middle_pos):
        if USE_DEGREE:
            middle_pos[2] = np.deg2rad(middle_pos[2])
        self.milestone = middle_pos
        
        if self.filt is not None:
            self.check_rot = True
            self.prev_time = time.time()
            self.prev_pos = middle_pos[0:2].reshape((-1, 1))

    def update_status(self, status):
        self.current_status = status
        self.history_status.append(status)

    def save_debug_data(self):
        print("saving " + self.save_name)
        if not SAVE_DEBUG: return
        with open('DEBUG/' + self.save_name + '.json', 'w') as outfile:
            json.dump(self.save_json, outfile)


if __name__ == "__main__":
    a = np.array([
        [-2.47128687, 35.96591863],
        [-3.12465444, 32.66349921],
        [-2.95947115, 29.57229674],
        [-4.88126408, 26.11762138],
        [-5.2841994,  22.94340914]
        ])

    x=np.array([-12.336162241672904, -11.20851700194011, -9.767633715758231, -7.879921799985004, -6.229842108338116]).reshape(-1, 1)
    y=np.array([18.373618777973732, 16.42025859825967, 12.656110572809851, 8.87503754520391, 5.291247217914645]).reshape(-1, 1)
    t0 = time.time()
    result = estimate_angle(x, y)
    t1 = time.time()
    print(np.rad2deg(result[0]))
    print(t1 - t0)
import numpy as np

from .utils import minimized_angle
from .Config import USE_DEGREE
import time

class ParticleFilter:
    def __init__(self, mean, covs, num_particles, alphas, beta, init_angle_available):
        self.alphas = alphas
        self.beta = beta
        self.init_angle_available = init_angle_available

        if USE_DEGREE:
            mean[2,0] = np.deg2rad(mean[2,0])

        if self.init_angle_available:
            self._init_mean = mean
            self._init_cov = covs
        else:
            self._init_mean = mean[0:2]
            self._init_cov = covs[0:2, 0:2]
        self.num_particles = num_particles
        self.reset()

    def reset(self):
        self.particles = np.zeros((self.num_particles, 3))
        if self.init_angle_available:
            for i in range(self.num_particles):
                self.particles[i, :] = np.random.multivariate_normal(
                    self._init_mean.ravel(), self._init_cov)
        else:
            for i in range(self.num_particles):
                self.particles[i, 0:2] = np.random.multivariate_normal(
                    self._init_mean.ravel(), self._init_cov)  
                self.particles[i, 2] = (np.random.rand() - 0.5)*2*np.pi

        self.weights = np.ones(self.num_particles) / self.num_particles

    def update(self, env, u, z):
        """Update the state estimate after taking an action and receiving a landmark
        observation.

        u: action
        z: landmark observation
        marker_id: landmark ID
        """
        # YOUR IMPLEMENTATION HERE
        #for i in range(0, self.num_particles):
        
        #print(self.particles.shape)
        Labmda = 0
        particle_after = []
        weight_after = []
        
        motion_ob = env.sample_noisy_actions(u, self.num_particles) 
        x = env.forward_multiple(self.particles, motion_ob)
        z_est = env.observe_mulitple(x)
        #print(z_est.shape)
        #print(z)
        for i in range(0, self.num_particles):   
            #z_est = env.observe(x[i, :])
            z_temp = z_est[i, :].reshape((-1, 1))
            w = env.likelihood( z_temp - z, self.beta)
            Labmda += w
            particle_after.append(x[i, :])
            weight_after.append(w)

        self.particles = np.array(particle_after) #np.concatenate(particle_after, axis=0)
        self.weights = np.array(weight_after)/Labmda
        #print(self.particles.shape, self.weights.shape)

        self.particles, self.weights = self.resample(self.particles, self.weights)
        #raise KeyboardInterrupt
        mean, cov = self.mean_and_variance(self.particles)

        return mean, self.particles

    def resample(self, particles, weights):
        """Sample new particles and weights given current particles and weights. Be sure
        to use the low-variance sampler from class.

        particles: (n x 3) matrix of poses
        weights: (n,) array of weights
        """
        new_particles, new_weights = particles, weights
        N = new_particles.shape[0]
        new_weights = new_weights/np.sum(new_weights)
        

        sum_weights = []
        for i in range(0, N):
            sum_weights.append(np.sum(new_weights[0:i+1]))
        sum_weights = np.array(sum_weights)

        rand_point = np.random.random()
        pick_index = []
        for i in range(0, N):
            if rand_point + i/N >= 1:
                point = rand_point + i/N - 1
            else:
                point = rand_point + i/N
            
            index = (sum_weights > point).argmax(axis=0)
            pick_index.append(index)

        new_particles, new_weights = particles[pick_index, :], weights[pick_index]
            

        # YOUR IMPLEMENTATION HERE
        return new_particles, new_weights

    def mean_and_variance(self, particles):
        """Compute the mean and covariance matrix for a set of equally-weighted
        particles.

        particles: (n x 3) matrix of poses
        """
        mean = particles.mean(axis=0)
        mean[2] = np.arctan2(
            np.sin(particles[:, 2]).sum(),
            np.cos(particles[:, 2]).sum(),
        )

        zero_mean = particles - mean
        for i in range(zero_mean.shape[0]):
            zero_mean[i, 2] = minimized_angle(zero_mean[i, 2])
        cov = np.dot(zero_mean.T, zero_mean) / self.num_particles

        return mean.reshape((-1, 1)), cov

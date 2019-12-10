import numpy as np
import numpy.linalg as la
import csv

class Kalman_3D():
    def __init__(self):
        self.acceleration = np.array([0, 0, -9.8])
        self.dt = 0
        self.F = np.array([1, 0, 0, self.dt, 0, 0,
              0, 1, 0, 0, self.dt, 0,
              0, 0, 1, 0, 0, self.dt,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1]).reshape(6,6) # dynamic system model
        self.B = np.array([self.dt**2/2, 0, 0,
              0, self.dt**2/2, 0,
              0, 0, self.dt**2/2,
              self.dt, 0, 0,
              0, self.dt, 0,
              0, 0, self.dt]).reshape(6,3) # acceleration model
        self.H = np.array([1,0,0,0,0,0,
              0,1,0,0,0,0,
              0,0,1,0,0,0]).reshape(3,6) # position only, no velocity involved
        self.initial_state = np.array([0,0,0,0,0,0]) # x, y, z, vx, vy, vz
        self.initial_covariance = np.diag([500, 500, 500, 500, 500, 500])**2
        self.Q = 0.0001**2 * np.eye(6) # dynamic system model noise
        self.noise = 0.1
        self.R = 3*self.noise**2 * np.eye(3) # observation model noise
        self.result = []
        self.observation = []
        self.max_iteration = 1000

    def update_matrix(self, time_difference):
        self.dt = time_difference
        self.F = np.array([1, 0, 0, self.dt, 0, 0,
              0, 1, 0, 0, self.dt, 0,
              0, 0, 1, 0, 0, self.dt,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1]).reshape(6,6) # dynamic system model
        self.B = np.array([self.dt**2/2, 0, 0,
              0, self.dt**2/2, 0,
              0, 0, self.dt**2/2,
              self.dt, 0, 0,
              0, self.dt, 0,
              0, 0, self.dt]).reshape(6,3) # acceleration model

    def kalman_filter(self, cur_state, cur_covariance, tracked_points):
        # F, Q : dynamic system model and its noise
        # H, R : observation model and its noise 
        # B : acceleration model

        state_predicted = self.F @ cur_state + self.B @ self.acceleration
        covariance_predicted  = self.F @ cur_covariance @ self.F.transpose() + self.Q
        if(tracked_points is None):
            return state_predicted, covariance_predicted
        
        # difference between observation and prediction
        epsilon = tracked_points - self.H @ state_predicted 
        kalman_gain = covariance_predicted @ self.H.transpose() @ la.inv(self.H @ covariance_predicted @ self.H.transpose() + self.R)
        new_state = state_predicted + kalman_gain @ epsilon
        new_covariance = (np.eye(len(cur_covariance))-kalman_gain @ self.H) @ covariance_predicted
        return new_state, new_covariance

    def prediction(self, tracked_points, time_difference):
        self.update_matrix(time_difference)
        self.result = []
        self.observation.append(tracked_points)
        new_state = self.initial_state
        new_covariance = self.initial_covariance
        counter = 0
        iteration_count = 0

        once = True
        while once or (new_state[2] >= 0.4 and iteration_count < self.max_iteration):
            once = False
            if(counter < len(self.observation)):
              new_state, new_covariance = self.kalman_filter(new_state, new_covariance, self.observation[counter]) 
              counter += 1
              if(counter == len(self.observation)):
                # Add the current tracke points' next state
                self.result.append(new_state)
            else:
              new_state, new_covariance = self.kalman_filter(new_state, new_covariance, None)
              self.result.append(new_state)
            iteration_count += 1
        return self.result #x, y, z, vx, vy, vz 

    def reset_state(self):
        self.initial_state = np.array([0,0,0,0,0,0]) # x, y, z, vx, vy, vz
        self.initial_covariance = np.diag([1000,1000,1000,1000,1000,1000])**2
        self.result = []
        self.observation = []



import numpy as np
import numpy.linalg as la
import cv2
import csv

def kalman(cur_state,cur_covariance,F,Q,B,acceleration,observation,H,R):
    # F, Q : dynamic system model and its noise
    # H, R : observation model and its noise 
    # B : acceleration model

    state_predicted = F @ cur_state + B @ acceleration
    print(state_predicted, cur_state)
    covariance_predicted  = F @ cur_covariance @ F.transpose() + Q
    if(observation is None):
        return state_predicted, covariance_predicted
    
    # difference between observation and prediction
    epsilon = observation - H @ state_predicted 
    kalman_gain = covariance_predicted @ H.transpose() @ la.inv(H @ covariance_predicted @ H.transpose() +R)
    new_state = state_predicted + kalman_gain @ epsilon
    new_covariance = (np.eye(len(cur_covariance))-kalman_gain @ H) @ covariance_predicted
    return new_state, new_covariance

## Initial parameters for Kalman filters, z acceleration
acceleration = np.array([0, 0, 1600])
fps = 102
dt = 1/fps

F = np.array([1, 0, 0, dt, 0, 0,
              0, 1, 0, 0, dt, 0,
              0, 0, 1, 0, 0, dt,
              0, 0, 0, 1, 0, 0,
              0, 0, 0, 0, 1, 0,
              0, 0, 0, 0, 0, 1]).reshape(6,6) # dynamic system model
B = np.array([dt**2/2, 0, 0,
              0, dt**2/2, 0,
              0, 0, dt**2/2,
              dt, 0, 0,
              0, dt, 0,
              0, 0, dt]).reshape(6,3) # acceleration model
H = np.array([1,0,0,0,0,0
              0,1,0,0,0,0,
              0,0,1,0,0,0]).reshape(3,6) # position only, no velocity involved
initial_state = np.array([0,0,0,0,0,0]) # x, y, z, vx, vy, vz
initial_covariance= np.diag([1000,1000,1000,1000,1000,1000])**2
res=[]

Q = 0.0001**2 * np.eye(6) # dynamic system model noise
noise = 3
R = 3*noise**2 * np.eye(3) # observation model noise

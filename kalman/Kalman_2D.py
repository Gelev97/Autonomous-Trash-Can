import numpy as np
import numpy.linalg as la
import cv2
import csv
import time

class Kalman_2D():
    def __init__(self):
        self.acceleration = np.array([0, 1600])
        self.dt = 0
        self.F = np.array([1, 0, self.dt, 0, 0, 1, 0, self.dt, 0, 0, 1, 0, 0, 0, 0, 1]).reshape(4,4) # dynamic system model
        self.B = np.array([self.dt**2/2, 0, 0, self.dt**2/2, self.dt, 0, 0, self.dt]).reshape(4,2) # acceleration model
        self.H =  np.array([1,0,0,0,0,1,0,0]).reshape(2,4) # position only, no velocity involved
        self.initial_state = np.array([0,0.1,0,0]) # x, y, vx, vy
        self.initial_covariance = np.diag([1000,1000,1000,1000])**2
        self.Q = 0.0001**2 * np.eye(4) # dynamic system model noise
        self.noise = 3
        self.R = 3*self.noise**2 * np.eye(2) # observation model noise
        self.result = []
        self.observation = []
        self.max_iteration = 1000

    def update_matrix(self, time_difference):
        self.dt = time_difference
        self.F = np.array([1, 0, self.dt, 0, 0, 1, 0, self.dt, 0, 0, 1, 0, 0, 0, 0, 1]).reshape(4,4) # dynamic system model
        self.B = np.array([self.dt**2/2, 0, 0, self.dt**2/2, self.dt, 0, 0, self.dt]).reshape(4,2) # acceleration model

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
        flag = False
        iteration_count = 0

        while(new_state[1] < 1080.0 and iteration_count < self.max_iteration):
            if(flag == False):
                new_state, new_covariance = self.kalman_filter(new_state, new_covariance, tracked_points) 
                flag = True
                self.initial_state = new_state
                self.initial_covariance = new_covariance
            else:
                new_state, new_covariance = self.kalman_filter(new_state, new_covariance, None)
            self.result.append(new_state)
            iteration_count += 1
        return self.result #x, y, z, vx, vy, vz 

    def reset_observation(self):
        self.observation = []

## Get video and hardcode coordination
video_path = "./First Clip.mov"
csv_file = "./Results.csv"
vidObj = cv2.VideoCapture(video_path) 
rows = []
with open(csv_file, 'r') as csvfile: 
    # creating a csv reader object 
    csvreader = csv.reader(csvfile) 
  
    # extracting each data row one by one 
    for row in csvreader: 
        rows.append(row)

coordinates_array = []
for row_index in range(1,len(rows)):
    coordinates_array.append((int(float(rows[row_index][5])),int(float(rows[row_index][6]))))

success = 1# checks whether frames were extracted 
count = 0

def main():
    kalman_obj = Kalman_2D()
    result_time = 0
    
    count = 0
    while(count < 151):
        success, image = vidObj.read()

        start = time.time()
        prediction = kalman_obj.prediction((coordinates_array[count][0], coordinates_array[count][1]), 1/102)
        end = time.time()

        result_time += end - start
        # draw tracked coordinates 
        for coordinate_tracked in kalman_obj.observation:
            cv2.circle(image, (int(coordinate_tracked[0]),int(coordinate_tracked[1])), 10, (0,255,0), 2)
        
        for coordinate_predicted in prediction:
            cv2.circle(image, (int(coordinate_predicted[0]),int(coordinate_predicted[1])), 10, (0,0,255), 2)
        
        cv2.imshow('image',image)
        cv2.waitKey(0)
        count += 1
    cv2.destroyAllWindows()
    kalman_obj.reset_observation()
    print(result_time)

main()


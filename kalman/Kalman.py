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

out = cv2.VideoWriter('./video1.avi',cv2.VideoWriter_fourcc('M','J','P','G'),20,(1920,1080))

## Initial parameters for Kalman filters
acceleration = np.array([0, 1600])
fps = 102
dt = 1/fps

F = np.array([1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1]).reshape(4,4) # dynamic system model
B = np.array([dt**2/2, 0, 0, dt**2/2, dt, 0, 0, dt]).reshape(4,2) # acceleration model
H = np.array([1,0,0,0,0,1,0,0]).reshape(2,4) # position only, no velocity involved
initial_state = np.array([0,0,0,0]) # x, y, vx, vy
initial_covariance= np.diag([1000,1000,1000,1000])**2
res=[]

Q = 0.0001**2 * np.eye(4) # dynamic system model noise
noise = 3
R = 3*noise**2 * np.eye(2) # observation model noise

def main():
    observed = []
    new_state = initial_state
    new_covariance = initial_covariance

    count = 0
    while(count < 151):
        success, image = vidObj.read()
        observed.append((coordinates_array[count][0], coordinates_array[count][1]))

        prediction = []
        for index in range(0,151):
            if(index <= count):
                new_state, new_covariance= kalman(new_state,new_covariance,F,Q,B,acceleration,np.array([observed[index][0], observed[index][1]]),H,R)
            else:
                new_state, new_covariance= kalman(new_state,new_covariance,F,Q,B,acceleration,None,H,R)
            prediction.append(new_state)
        new_state = initial_state
        new_covariance = initial_covariance
    
        # draw tracked coordinates 
        for coordinate_tracked in observed:
            cv2.circle(image, (int(coordinate_tracked[0]),int(coordinate_tracked[1])), 10, (0,255,0), 2)
        
        for coordinate_predicted in prediction:
            cv2.circle(image, (int(coordinate_predicted[0]),int(coordinate_predicted[1])), 10, (0,0,255), 2)
        
        cv2.imshow('image',image)
        out.write(image)
        cv2.waitKey(0)
        count += 1
    out.release()
    cv2.destroyAllWindows()

main()


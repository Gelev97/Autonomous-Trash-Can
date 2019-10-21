import numpy as np
import cv2
import csv
import time

video_path = "./First Clip.mov"
vidObj = cv2.VideoCapture(video_path) 

success = 1# checks whether frames were extracted 
count = 0

# Calculate fps
start = time.time()
for i in range(0, 150) :
    ret, frame = vidObj.read()
end = time.time()
seconds = end - start
fps  = 150 / seconds;
print(fps)

vidObj = cv2.VideoCapture(video_path) 

while(success == 1):
    success, image = vidObj.read()
    cv2.imshow('image',image)
    cv2.imwrite('image_' + str(count) + '.png',image)
    count += 1
    cv2.waitKey(0)
out.release()
cv2.destroyAllWindows()



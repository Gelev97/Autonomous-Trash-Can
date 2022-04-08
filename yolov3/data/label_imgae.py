import cv2
import numpy as np

# for number in range(0, 765):
#     img = cv2.imread('./train/frame' + str(number) +'.jpg')
#     height, width = (img.shape[0], img.shape[1])
#     f= open("./labels/train/" + str(number) +'.txt',"w+")
#     r = cv2.selectROI(img)
#     (center_x_raw, center_y_raw, width_raw, height_raw) = (r[0]+r[2]/2,r[1]+r[3]/2,r[2],r[3])
#     (center_x, center_y, width, height) = (center_x_raw/width, center_y_raw/height, width_raw/width, height_raw/height)

#     print(center_x, center_y, width, height)

#     f.write("0 " + str(center_x) + " " + str(center_y) + " " + str(width) + " " + str(height))

#     f.close()

f= open('train.txt',"w+")
for number in range(0, 765):      
    f.write("data/images/train/" + str(number) +'.jpg\n')
f.close()

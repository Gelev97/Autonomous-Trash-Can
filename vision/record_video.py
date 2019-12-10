from realsense import *
import cv2
import numpy as np

out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc("M", "J", "P", "G"), 30, (640, 360))
camera = realsense()

while(True):
	color_frame, depth_frame = camera.get_rs_frames()
	color_image = np.asanyarray(color_frame.get_data())
	out.write(color_image)

	cv2.imshow('frame', color_image)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

out.release()
del camera
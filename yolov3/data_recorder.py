import cv2
import numpy as np
from realsense import *
import sys

def main():
    name = sys.argv[1]
    # Create a VideoCapture object
    camera = realsense()

    color_frame, depth_frame = camera.get_rs_frames()
    img = np.asanyarray(color_frame.get_data())

    frame_width = int(img.shape[1])
    frame_height = int(img.shape[0])
    print(frame_width,frame_height)
    # Default resolutions of the frame are obtained.The default resolutions are system dependent.
    # We convert the resolutions from float to integer.
    # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.

    out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

    while(True):
        color_frame, depth_frame = camera.get_rs_frames()
        img = np.asanyarray(color_frame.get_data())
        out.write(img)
        cv2.imshow('frame',img)
        # Press Q on keyboard to stop recording
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    out.release()
    cv2.destroyAllWindows() 

main()
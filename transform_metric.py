from vision.realsense import *
from kalman.Kalman_3D import *
from vision.camshift import *
from vision.yolo.detect import *
from vision.transform import transform
import numpy as np
import time
import cv2

def main():
    camera = realsense()
    yolo = yolo_tracking()

    while True:
        color_frame, depth_frame = camera.get_rs_frames()
        color_image = np.asanyarray(color_frame.get_data())

        img = yolo.parse_img(color_image)
        center_pixels = yolo.detect(img)            
        if len(center_pixels) == 1:
            center_pixel = center_pixels[0]
            color_image = cv2.circle(color_image, center_pixel, 10, (255, 0, 0), 4)
            pixel_3d = camera.cord_3d(list(center_pixel))
            transformed_3d = transform(pixel_3d)
            print(transformed_3d)
        cv2.namedWindow("transform", cv2.WND_PROP_FULLSCREEN)
        cv2.resizeWindow("transform", 1280, 720)
        cv2.imshow('transform', color_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()
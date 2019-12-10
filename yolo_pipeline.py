from vision.realsense import realsense
from kalman.Kalman_3D import Kalman_3D
from vision.yolo.detect import yolo_tracking
from vision.transform import transform, back_transform
from connection.bt import connection
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import threading
import math
import os

def close(a, b):
    return abs(a[0] - b[0]) < 0.05 and abs(a[1]-b[1]) < 0.05 and abs(a[2]-b[2]) < 0.05

def far(a, b):
    return abs(a[0] - b[0]) > 0.5 or abs(a[1]-b[1]) > 0.5 or abs(a[2]-b[2]) > 0.5

def close_2d(a, b):
    return abs(a[0] - b[0]) < 0.15 and abs(a[1]-b[1]) < 0.15

def proportion_point(a, b):
    proportion = (a[2]-0.4)/(a[2]-b[2])
    return [a[0]+(b[0]-a[0])*proportion, a[1]+(b[1]-a[1])*proportion, 0.4]

class yolo_pipeline:
    def __init__(self, bt=True):
        self.camera = realsense()
        self.final_pos = None

        self.start = time.time()
        self.kalman = Kalman_3D()
        self.yolo = yolo_tracking()
        self.previous_pos = None
        self.fail_counter = 0
        self.disappear_counter = 0
        self.enable_bt = bt
        self.record = False
        if self.enable_bt:
            self.bluetooth = connection()
            self.bluetooth.send(30, 30)
        self.file = None
        # self.all_predictions = []

    def back(self):
        # self.bluetooth.send(self.x, 0)
        # time.sleep(2)
        if self.enable_bt:
            self.bluetooth.send(30, 30)

    def setRecord(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.record = not self.record
            if self.record:
                self.file = open("log.txt", "a")
                self.file.write("Trial:\n")
            else:
                self.file.close()

    def loop(self):
        # t = time.time()
        color_frame, depth_frame = self.camera.get_rs_frames()
        color_image = np.asanyarray(color_frame.get_data())
        # print("Camera time: " + str(time.time()-t))
        # t = time.time()
        img = self.yolo.parse_img(color_image)
        center_pixels = self.yolo.detect(img)
        # print("Yolo time: " + str(time.time()-t))
        # t = time.time()
        if len(center_pixels) == 1:
            self.disappear_counter = 0
            center_pixel = center_pixels[0]
            color_image = cv2.circle(color_image, center_pixel, 10, (255, 0, 0), 4)
            pixel_3d = self.camera.cord_3d(list(center_pixel))
            transformed_3d = transform(pixel_3d)
            if self.record:
                self.file.write("[{}, {}, {}]\n".format(transformed_3d[0], transformed_3d[1], transformed_3d[2]))
            if self.previous_pos is None or not (close(self.previous_pos, transformed_3d) or far(self.previous_pos, transformed_3d)):
                time_diff = time.time()-self.start
                self.start = time.time()
                prediction_list = self.kalman.prediction(transformed_3d, time_diff)
                for coordinate_predicted in prediction_list:
                    pixel_3d = back_transform(coordinate_predicted)
                    pixel_2d = self.camera.point_projection(pixel_3d)
                    # print([coordinate_predicted[0], coordinate_predicted[1], coordinate_predicted[2]])
                    cv2.circle(color_image, (int(pixel_2d[0]), int(pixel_2d[1])), 10, (0,0,255), 2)
                # print("Kalman time: " + str(time.time()-t))
                if len(prediction_list) <= 1:
                    self.kalman.reset_state()
                if len(prediction_list) > 1:
                    # self.all_predictions.append(prediction_list)
                    point4 = proportion_point(prediction_list[-2], prediction_list[-1])
                    # final_pos = prediction_list[-1]
                    final_pos = point4
                    if self.final_pos is None:
                        self.final_pos = final_pos
                    elif not close_2d(final_pos, self.final_pos):
                        self.final_pos = final_pos
                        self.fail_counter += 1
                        if self.fail_counter >= 4:
                            self.kalman.reset_state()
                            print("Not parabolic, reset")
                            self.fail_counter = 0
                    else:
                        self.final_pos = final_pos
                        x = self.final_pos[0]
                        y = self.final_pos[1]
                        x = x + 0.05
                        print(x, y)
                        if x > 0.0 and y > 0.0 and x ** 2 + y ** 2 < 1:
                            x = int(x*100)-10 if x >= 0.1 else 0
                            y = int(y*100)-10 if y >= 0.1 else 0
                            print("Go")
                            if self.record:
                                self.file.write("Go\n")
                            if self.enable_bt:
                                self.bluetooth.send(x, y)
                                threading.Timer(2, self.back).start()
                        else:
                            print("Not inside circle")
                self.previous_pos = transformed_3d
            elif close(self.previous_pos, transformed_3d):
                print("Not moving")
                self.kalman.reset_state()
                self.previous_pos = transformed_3d
            elif far(self.previous_pos, transformed_3d):
                print("Wrong observation")
        else:
            self.disappear_counter += 1
            if self.disappear_counter == 3:
                print("No ball")
                self.kalman.reset_state()
                self.previous_pos = None
                self.final_pos = None
        # print("Total time: " + str(time.time()-t))

        cv2.namedWindow("detect", cv2.WND_PROP_FULLSCREEN)
        cv2.resizeWindow("detect", 1280, 720)
        cv2.imshow('detect', color_image)
        cv2.setMouseCallback("detect", self.setRecord)
        cv2.waitKey(1)

    def __del__(self):
        del self.camera
        # np.save("predictions", np.array(self.all_predictions))
        if self.enable_bt:
            del self.bluetooth
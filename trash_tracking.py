import sys
from vision.realsense import *
from kalman.Kalman_3D import *
from vision.camshift import *
import numpy as np
import time
import cv2

def close(a, b):
    return abs(a[0] - b[0]) < 0.2 and abs(a[1]-b[1]) < 0.2 and abs(a[2]-b[2]) < 0.2

class trash_tracking():
    def __init__(self):
        self.camera = realsense()
        self.fgbg = cv2.bgsegm.createBackgroundSubtractorGMG()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        self.prediction_list = None

        self.bounding_box = None
        self.start_track = False
        self.track_phase = False
        self.start = time.time()
        self.kalman = Kalman_3D()
        self.fail_counter = 0
        self.success_counter = 0

    def remove_background(self, color_image):
        cv_grey = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        fgmask = self.fgbg.apply(cv_grey)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self.kernel)
        
        kernel = np.ones((3,3),np.uint8)
        # fgmask = cv2.erode(fgmask,kernel,iterations = 1)
        fgmask = cv2.medianBlur(fgmask,3)
        fgmask = cv2.dilate(fgmask,kernel,iterations=2)
        fgmask = (fgmask > 200).astype(np.uint8)*255
        return fgmask

    def get_components(self, fgmask):
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(fgmask, ltype=cv2.CV_16U)
        min_area = 700

        good_stats = []
        for stat in stats:
            area = stat[cv2.CC_STAT_AREA]
            width = stat[cv2.CC_STAT_WIDTH]
            height = stat[cv2.CC_STAT_HEIGHT]
            if area < min_area or (width/height > 1.5 or width/height < 0.67):
                continue # Skip small objects (noise)
            if width == 640 and height == 480:
                continue # ignore the entire frame

            good_stats.append(stat)
                
            lt = (stat[cv2.CC_STAT_LEFT], stat[cv2.CC_STAT_TOP])
            rb = (lt[0] + width, lt[1] + height)                                                
            bottomLeftCornerOfText = (lt[0], lt[1] - 15)

            cv2.rectangle(fgmask, lt, rb, 127, 1)

            cv2.putText(fgmask, "{}: {:.0f}".format(len(good_stats), stat[cv2.CC_STAT_AREA]),
                        bottomLeftCornerOfText, 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, 127, 1)
        if len(good_stats) == 1:
            (x, y, w, h) = [good_stats[0][cv2.CC_STAT_LEFT], good_stats[0][cv2.CC_STAT_TOP], good_stats[0][cv2.CC_STAT_WIDTH], good_stats[0][cv2.CC_STAT_HEIGHT]]
            self.bounding_box = (x, y, w, h)
            if not self.track_phase:
                self.start_track = True
            else:
                self.start_track = False
            self.track_phase = True
        else:
            self.bounding_box = None

    
    def loop(self):
        color_frame, depth_frame = self.camera.get_rs_frames()
        color_image = np.asanyarray(color_frame.get_data())

        fgmask = self.remove_background(color_image)
        color_mask = cv2.bitwise_and(color_image, color_image, mask = fgmask)

        self.get_components(fgmask)
        
        if self.bounding_box is not None:
            time_diff = time.time()-self.start
            self.start = time.time()
            (x, y, w, h) = [int(v) for v in self.bounding_box]
            cv2.rectangle(color_mask, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # # print(self.bounding_box)
        # if self.start_track:
        #     self.tracker.init(color_mask, self.bounding_box)
        # if self.track_phase:
        #     (success, self.bounding_box) = self.tracker.update(color_mask)
        #     if not success:
        #         self.track_phase = False
        #         print("Nah")
        #     if success:
        #         (x, y, w, h) = [int(v) for v in self.bounding_box]
        #         cv2.rectangle(color_mask, (x, y), (x + w, y + h),
        #             (0, 255, 0), 2)
        #         print("Yeah")

            center_pixel = [int(x + w/2), int(y + h/2)]
            pixel_3d = self.camera.cord_3d(center_pixel)
            if self.prediction_list is not None:
                if not close(pixel_3d, self.prediction_list[0]):
                    self.fail_counter += 1
                    self.success_counter = 0
                    if self.fail_counter >= 3:
                        self.kalman.reset_state()
                        print("reset")
                else:
                    self.fail_counter = 0
                    self.success_counter += 1
                if self.success_counter >= 7:
                    print("yeah")
            self.prediction_list = self.kalman.prediction(pixel_3d, time_diff)
            for coordinate_predicted in self.prediction_list:
                if not ((coordinate_predicted[0] == 0 and coordinate_predicted[1] == 0) or coordinate_predicted[2] == 0):
                    pixel_2d = self.camera.point_projection([coordinate_predicted[0], coordinate_predicted[1], coordinate_predicted[2]])
                    # print([coordinate_predicted[0], coordinate_predicted[1], coordinate_predicted[2]])
                    cv2.circle(color_mask, (int(pixel_2d[0]), int(pixel_2d[1])), 10, (0,0,255), 2)
                
            # for coordinate_tracked in self.kalman.observation:
            #     if not (coordinate_tracked[0] == 0 and coordinate_tracked[1] == 0 and coordinate_tracked[2] == 0):
            #         pixel_2d = self.camera.point_projection([coordinate_tracked[0], coordinate_tracked[1], coordinate_tracked[2]])
            #         cv2.circle(color_mask, (int(pixel_2d[0]), int(pixel_2d[1])), 10, (0,255,0), 2)
        
        cv2.namedWindow('Color Image', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Color Image', color_mask)
        cv2.namedWindow('GMG', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('GMG', fgmask)
        cv2.waitKey(1)

    def __del__(self):
        del self.camera
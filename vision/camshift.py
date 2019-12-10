import numpy as np
import cv2

class camshift():
    def __init__(self):
        self.roi_hist = None
        self.term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        self.track_window = None
    
    def reset_camshift(self, color_mask, bounding_box):
        self.track_window = bounding_box
        (x, y, w, h) = bounding_box
        roi = color_mask[y:y+h, x:x+w]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
        self.roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
        cv2.normalize(self.roi_hist,self.roi_hist,0,255,cv2.NORM_MINMAX)

    def track(self, color_mask):
        hsv = cv2.cvtColor(color_mask, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv],[0],self.roi_hist,[0,180],1)
        print(self.track_window)
        ret, self.track_window = cv2.CamShift(dst, self.track_window, self.term_crit)
        # Draw it on image
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        color_mask = cv2.polylines(color_mask,[pts],True, 255,2)
import pyrealsense2 as rs
import time
import numpy as np
import cv2


# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

# Start streaming
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

fgbg = cv2.createBackgroundSubtractorMOG2()
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))

tracker = cv2.TrackerMIL_create()
initBB = None
track_phase = False

kernel = np.ones((3,3),np.uint8)

# Streaming loop
try:
    while True:
        start = time.time()
    	# Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image
        
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image = depth_image*depth_scale

        color_image = np.asanyarray(color_frame.get_data())

        if not track_phase:
            cv_grey = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            fgmask = fgbg.apply(cv_grey)
            fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
            
            fgmask = cv2.erode(fgmask,kernel,iterations = 1)
            fgmask = cv2.medianBlur(fgmask,3)
            fgmask = cv2.dilate(fgmask,kernel,iterations=2)
            fgmask = (fgmask > 200).astype(np.uint8)*255
            colorMask = cv2.bitwise_and(color_image, color_image, mask = fgmask)

            num, labels, stats, centroids = cv2.connectedComponentsWithStats(fgmask, ltype=cv2.CV_16U)
            candidates = list()
            min_area = 500

            for stat in stats:
                area = stat[cv2.CC_STAT_AREA]
                if area < min_area:
                    continue # Skip small objects (noise)
                    
                lt = (stat[cv2.CC_STAT_LEFT], stat[cv2.CC_STAT_TOP])
                rb = (lt[0] + stat[cv2.CC_STAT_WIDTH], lt[1] + stat[cv2.CC_STAT_HEIGHT])                                                
                bottomLeftCornerOfText = (lt[0], lt[1] - 15)

                candidates.append((lt, rb, area))
                cv2.rectangle(fgmask, lt, rb, 127, 1)

                cv2.putText(fgmask, "{}: {:.0f}".format(len(candidates), stat[cv2.CC_STAT_AREA]),
                            bottomLeftCornerOfText, 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, 127, 1)
            good_stats = [stat for stat in stats if stat[cv2.CC_STAT_AREA] > min_area]

        
            if len(good_stats) == 2:
                initBB = (stats[1][cv2.CC_STAT_LEFT], stats[1][cv2.CC_STAT_TOP], stats[1][cv2.CC_STAT_WIDTH], stats[1][cv2.CC_STAT_HEIGHT])
                (x, y, w, h) = [int(v) for v in initBB]
                cv2.rectangle(colorMask, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.namedWindow('Hough Circle', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Hough Circle', colorMask)
        cv2.namedWindow('MOG2', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('MOG2', fgmask)
        cv2.waitKey(1)

finally:
	pipeline.stop()

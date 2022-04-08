import cv2

VIDEO_NAME = "IMG_7797.MOV"
count = 352

vidcap = cv2.VideoCapture(VIDEO_NAME)
success,image = vidcap.read()

while success:
  cv2.imwrite("./images/train/%d.jpg" % count, image)     # save frame as JPEG file      
  success,image = vidcap.read()
  count += 1

print(count - 1)

import argparse
from sys import platform

from models import *  # set ONNX_EXPORT in models.py
from utils.datasets import *
from utils.utils import *
from realsense import *
import cv2 as cv


def detect(weights, cfg, names, im0, bottle):
    img_size = 416 

    # Initialize
    device = torch_utils.select_device(device='cpu' if ONNX_EXPORT else '')

    # Initialize model
    model = Darknet(cfg, img_size)

    # Load weights
    attempt_download(weights)
    if weights.endswith('.pt'):  # pytorch format
        model.load_state_dict(torch.load(weights, map_location=device)['model'])
    else:  # darknet format
        load_darknet_weights(model, weights)

    # Eval mode
    model.to(device).eval()
  
    # Get names and colors
    names = load_classes(names)
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

    # Run inference
    img = letterbox(im0, new_shape=416)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    pred = model(img)[0]

    # Apply NMS
    pred = non_max_suppression(pred, 0.3, 0.6)

    # Process detections
    for i, det in enumerate(pred):  # detections per image
        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in det:
                if(bottle):
                    if(cls == 39):
                        label = '%s %.2f' % (names[int(cls)], conf)
                        plot_one_box(xyxy, im0, label=label, color=colors[int(cls)])
                else:
                    label = '%s %.2f' % (names[int(cls)], conf)
                    plot_one_box(xyxy, im0, label=label, color=colors[int(cls)])

    return im0

if __name__ == '__main__':
    camera = realsense()

    # img = cv.imread('./data/samples/unnamed1.jpg')
    with torch.no_grad():
        while(1):
            color_frame, depth_frame = camera.get_rs_frames()
            img = np.asanyarray(color_frame.get_data())
            im0 = detect('weights/best.pt','cfg/yolov3-tiny.cfg','data/train.names',img, False)
            im0 = detect('weights/yolov3-spp-ultralytics.pt','cfg/yolov3-spp.cfg','data/coco.names',im0, True)

            cv2.namedWindow("detect", cv2.WND_PROP_FULLSCREEN)
            cv2.resizeWindow("detect", 1280, 720)
            cv2.imshow('detect',im0)
            cv2.waitKey(1)

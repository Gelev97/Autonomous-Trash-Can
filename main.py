import argparse
from trash_tracking import *
from yolo_pipeline import *

# tracking = trash_tracking()
parser = argparse.ArgumentParser(description='18500')
parser.add_argument('-b', action='store_true')
args = parser.parse_args()
tracking = yolo_pipeline(not args.b)

try:
	while True:
		tracking.loop()
finally:
	del tracking
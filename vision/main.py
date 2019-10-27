from trash_tracking import *

tracking = trash_tracking()

try:
	while True:
		tracking.loop()
finally:
	del tracking
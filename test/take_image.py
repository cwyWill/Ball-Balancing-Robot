
from picamera2 import Picamera2, Preview
import time
from pprint import *

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)

# pprint(camera_config)

# pprint(picam2.sensor_modes)

# picam2.start_preview(Preview.QTGL)
picam2.start(show_preview=False)

time.sleep(2)
picam2.capture_file("test.jpg")

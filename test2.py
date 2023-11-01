from pyvjoystick import vjoy
import time
import cv2
import numpy as np
import random
from pose_estmation import get_pose
# Pythonic API, item-at-a-time
j = vjoy.VJoyDevice(1)

calibration_min = 0x0
calibration_max = 0x8000
j.reset()
j.reset_buttons()
j.reset_povs()

def calibrate(calibration_min, calibration_max, time_to_sleep=0.1):
    for v in [calibration_min, calibration_max, int(calibration_max/2)]:
        j.set_axis(vjoy.HID_USAGE.X, v)
        time.sleep(time_to_sleep)
        j.set_axis(vjoy.HID_USAGE.Y, v)
        time.sleep(time_to_sleep)
        j.set_axis(vjoy.HID_USAGE.Z, v)
        time.sleep(time_to_sleep)
        j.set_axis(vjoy.HID_USAGE.RX, v)
        time.sleep(time_to_sleep)
        j.set_axis(vjoy.HID_USAGE.RY, v)
        time.sleep(time_to_sleep)
        j.set_axis(vjoy.HID_USAGE.RZ, v)
        time.sleep(time_to_sleep)
        j.set_axis(vjoy.HID_USAGE.SL0, v)
        time.sleep(time_to_sleep)
        j.set_axis(vjoy.HID_USAGE.SL1, v)
        time.sleep(time_to_sleep)
        time.sleep(1)
# calibrate(calibration_min, calibration_max)


while True:
    x, y = get_pose()
    if not (np.isnan(x) or np.isnan(y)):
        vx = int(x*calibration_max)
        vy = int(y*calibration_max)
        print(f"Send :{vx}-{vy}")
        j.set_axis(vjoy.HID_USAGE.X, vx)
        time.sleep(0.01)
        j.set_axis(vjoy.HID_USAGE.Y, vy)
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()
    # j.update()





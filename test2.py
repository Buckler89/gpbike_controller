from pyvjoystick import vjoy
import time
import cv2
import numpy as np
import random
from pose_estmation import get_pose

import serial
import time

# Replace with your serial port
# arduino_port = "/dev/ttyACM0"  # Serial port of Arduino
arduino_port = "COM3"  # Serial port of Arduino
baud = 9600  # Baud rate to match the Arduino's

# Set up the serial connection
ser = serial.Serial(arduino_port, baud, timeout=1)
time.sleep(2)  # Wait for the connection to establish

# Initialize variables to store data


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

throttle = 0
front_brake = 0
rear_brake = 0
clutch = 0
def read_and_parse_data(serial_connection, throttle, front_brake, rear_brake, clutch):

    try:
        # Read a line from the serial port and decode it to UTF-8
        line = serial_connection.readline().decode('utf-8').rstrip()
        # Split the line into the prefix and the value
        if line:
            prefix, value_str = line[0], line[1:]
            value = int(value_str)

            # Assign the value to the correct variable
            if prefix == "T":
                throttle = value
            elif prefix == "F":
                front_brake = value
            elif prefix == "R":
                rear_brake = value
            elif prefix == "C":
                clutch = value

            # Print the values for debugging
            print(f"Throttle: {throttle}, Front Brake: {front_brake}, Rear Brake: {rear_brake}, Clutch: {clutch}")
    except ValueError as e:
        # If there's a ValueError, it's possible the line was incomplete or corrupted.
        print("Value error with line")
        print(e)
    except serial.SerialTimeoutException:
        print('Data could not be read')
        # time.sleep(1)
    return throttle, front_brake, rear_brake, clutch


x, y = 0, 0
throttle, front_brake, rear_brake, clutch = 0, 0, 0, 0
while True:
    # Start reading and parsing data
    try:
        # x, y = get_pose()
        throttle, front_brake, rear_brake, clutch = read_and_parse_data(ser, throttle, front_brake, rear_brake, clutch)
    except KeyboardInterrupt:
        # Close the serial connection if the script is stopped
        print("KeyboardInterrupt has been caught. Closing serial connection.")
        ser.close()
        break
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








import time
from pprint import pprint

from pyvjoystick import vjoy
from ultralytics import YOLO
import requests
import cv2
import numpy as np
import imutils
import serial
from multiprocessing import Process, Manager, Value, Queue
from ctypes import c_double, c_int


class ArduinoReader(Process):
    def __init__(self, queue, *args, **kwargs
                 ):
        super().__init__(*args, **kwargs)

        # arduino_port = "/dev/ttyACM0"  # Serial port of Arduino
        self.arduino_port = "COM3"  # Serial port of Arduino
        self.baud = 9600  # Baud rate to match the Arduino's

        self.ser = None
        self.queue = queue
        self.throttle = 0
        self.front_brake = 0
        self.rear_brake = 0
        self.clutch = 0

    def read_arduino_sensor(self):

        try:
            if not self.ser:
                # Set up the serial connection
                self.ser = serial.Serial(self.arduino_port, self.baud, timeout=1)
                time.sleep(2)  # Wait for the connection to establish
            # Read a line from the serial port and decode it to UTF-8
            line = self.ser.readline().decode('utf-8').rstrip()
            # Split the line into the prefix and the value
            if line:
                prefix, value_str = line[0], line[1:]
                value = int(value_str)

                # Assign the value to the correct variable
                if prefix == "T":
                    self.throttle = value
                elif prefix == "F":
                    self.front_brake = value
                elif prefix == "R":
                    self.rear_brake = value
                elif prefix == "C":
                    self.clutch = value

                # Print the values for debugging
                # print(f"Throttle: {self.throttle}, Front Brake: {self.front_brake}, Rear Brake: {self.rear_brake}, Clutch: {self.clutch}")
        except ValueError as e:
            # If there's a ValueError, it's possible the line was incomplete or corrupted.
            print("Value error with line")
            print(e)
        except serial.SerialTimeoutException:
            print('Data could not be read')
            # time.sleep(1)
        except KeyboardInterrupt:
            # Close the serial connection if the script is stopped
            print("KeyboardInterrupt has been caught. Closing serial connection.")
            self.ser.close()

    def run(self):
        while True:
            self.read_arduino_sensor()
            state = dict(
                        throttle=self.throttle,
                        front_brake=self.front_brake,
                        rear_brake=self.rear_brake,
                        clutch=self.clutch
                 )
            self.queue.put(state)

class PoseEstimator(Process):
    def __init__(self, queue, calibration_max):
        super().__init__()

        self.queue = queue
        self.image_width = 1000
        self.image_height = 1800
        self.image_url = "http://192.168.1.3:8080/shot.jpg"
        self.model = YOLO('yolov8n-pose.pt')  # load an official model

        self.calibration_max = calibration_max

        self.x = 0
        self.y = 0
        self.vx = 0
        self.vy = 0
    def get_pose(self):
        try:
            start_time = time.time()
            img_resp = requests.get(self.image_url)
            img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
            img = cv2.imdecode(img_arr, -1)
            # cv2.imshow("Android_cam", img)
            img = imutils.resize(img, width=self.image_width, height=self.image_height)
            # Predict with the model
            # results = model("http://192.168.1.3:8080/shot.jpg")  # predict on an image
            model_time = time.time()
            results = self.model(
                img, conf=0.7,
                max_det=1  # one person at time
            )  # predict on an image
            plot_time = time.time()
            print(f"model time: {plot_time - model_time}")
            result0 = results[0]
            annotated_frame = result0.plot()
            print(f"plot time: {time.time() - plot_time}")

            annotated_frame = imutils.resize(annotated_frame, width=self.image_width, height=self.image_height)

            result0 = result0.numpy()
            head_points = result0.keypoints.data[:, :5]
            head_points_norm = np.copy(head_points)
            head_points_norm[0, :, 0:2] = result0.keypoints.xyn[:, :5]

            condition = head_points_norm[:, :, 2] > 0.5
            average = np.mean(head_points_norm[condition], axis=0)

            end_time = time.time()
            fps = 1 / (end_time - start_time)
            print("FPS :", fps)

            cv2.putText(annotated_frame, "FPS :" + str(int(fps)), (10, 50), cv2.FONT_HERSHEY_COMPLEX, 1.2,
                        (255, 0, 255), 1,
                        cv2.LINE_AA)
            if not np.isnan(average).any():
                x_coord = int(average[0] * annotated_frame.shape[1])
                y_coord = int(average[1] * annotated_frame.shape[1])
                cv2.circle(annotated_frame, (x_coord, y_coord), 5, [0, 255, 0], -1, lineType=cv2.LINE_AA)
            cv2.imshow("YOLOv8 Inference", annotated_frame)
        except requests.exceptions.ConnectionError as e:
            print(e)
            return float("nan"), float("nan")
        self.x = average[0]
        self.y = average[1]
        if not (np.isnan(self.x) or np.isnan(self.y)):
            self.vx = int(self.x * self.calibration_max)
            self.vy = int(self.y * self.calibration_max)
            print(f"Send :{self.vx}-{self.vy}")
        # self.shared_state['x'] = self.x
        # self.shared_state['y'] = self.y
        # self.shared_state['vx'] = self.vx
        # self.shared_state['vy'] = self.vy

    def run(self):
        while True:
            self.get_pose()
            state = dict(
                x=self.x,
                y=self.y,
                vx=self.vx,
                vy=self.vy
            )
            self.queue.put(state)
            if cv2.waitKey(1) == 27:
                break

class GPBikeController:
    def __init__(self, queue):
        #
        self.controller = vjoy.VJoyDevice(1)
        self.controller.reset()
        self.controller.reset_buttons()
        self.controller.reset_povs()
        self.queue = queue
        self.calibration_min = 0x0
        self.calibration_max = 0x8000

        # manager = Manager()
        # self.shared_state = manager.dict()
        # self.shared_state['x'] = 0
        # self.shared_state['y'] = 0
        # self.shared_state['vx'] = 0
        # self.shared_state['vy'] = 0
        # self.shared_state['throttle'] = 0
        # self.shared_state['front_brake'] = 0
        # self.shared_state['rear_brake'] = 0
        # self.shared_state['clutch'] = 0
        self.throttle = 0
        self.front_brake = 0
        self.rear_brake = 0
        self.clutch = 0
        self.x = 0
        self.y = 0
        self.vx = 0
        self.vy = 0
        self.commands = dict(
            throttle=0,
            front_brake=0,
            rear_brake=0,
            clutch=0,
            x=0,
            y=0,
            vx=0,
            vy=0,
        )

    def calibrate(self, time_to_sleep=0.1):
        for v in [self.calibration_min, self.calibration_max, int(self.calibration_max / 2)]:
            self.controller.set_axis(vjoy.HID_USAGE.X, v)
            time.sleep(time_to_sleep)
            self.controller.set_axis(vjoy.HID_USAGE.Y, v)
            time.sleep(time_to_sleep)
            self.controller.set_axis(vjoy.HID_USAGE.Z, v)
            time.sleep(time_to_sleep)
            self.controller.set_axis(vjoy.HID_USAGE.RX, v)
            time.sleep(time_to_sleep)
            self.controller.set_axis(vjoy.HID_USAGE.RY, v)
            time.sleep(time_to_sleep)
            self.controller.set_axis(vjoy.HID_USAGE.RZ, v)
            time.sleep(time_to_sleep)
            self.controller.set_axis(vjoy.HID_USAGE.SL0, v)
            time.sleep(time_to_sleep)
            self.controller.set_axis(vjoy.HID_USAGE.SL1, v)
            time.sleep(time_to_sleep)
            time.sleep(1)

    def log_all_sensor(self):
        for v in enumerate([
                            self.throttle,
                            self.front_brake,
                            self.rear_brake,
                            self.clutch,
                            self.x,
                            self.y,
                            self.vx,
                            self.vy,
                            ]):
            print(v)
        print("----------------")

    def get_shared_state(self):
        new_commands = self.queue.get()
        self.commands.update(new_commands)
        print(self.commands)

    def write_all(self):
        self.controller.set_axis(vjoy.HID_USAGE.X, self.vx)
        time.sleep(0.01)  # necessario altrimeni non aggiorna i valori (usa update invece)
        self.controller.set_axis(vjoy.HID_USAGE.Y, self.vy)
        # todo write other data

    def runner(self):
        while True:
            try:
                self.get_shared_state()
                self.write_all()
            except Exception as e:
                print(e)

# def print_hi(name):
#     # Use a breakpoint in the code line below to debug your script.
#     print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
#
#     # Pythonic API, item-at-a-time
#
#     j = pyvjoy.VJoyDevice(1)
#
#     for i in range(20):
#         i=i+1
#         print(f"b: {i}")
#         j.set_button(i, 1)
#         time.sleep(1)
#     # turn button number 15 on
#     j.set_button(15, 1)
#
#     # Notice the args are (buttonID,state) whereas vJoy's native API is the other way around.
#
#     # turn button 15 off again
#     j.set_button(15, 0)
#
#     # Set X axis to fully left
#     j.set_axis(pyvjoy.HID_USAGE_X, 0x1)
#
#     # Set X axis to fully right
#     j.set_axis(pyvjoy.HID_USAGE_X, 0x8000)
#
#     # Also implemented:
#
#     j.reset()
#     j.reset_buttons()
#     j.reset_povs()
#
#     j.data.lButtons = 19  # buttons number 1,2 and 5 (1+2+16)
#     j.data.wAxisX = 0x2000
#     j.data.wAxisY = 0x7500
#
#     # send data to vJoy device
#     j.update()
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    queue = Queue()

    # print_hi('PyCharm')
    c = GPBikeController(queue)

    arduino_reader = ArduinoReader(
        queue
    )
    pose_estimator = PoseEstimator(queue, calibration_max=c.calibration_max)
    arduino_reader.start()
    pose_estimator.start()
    # c.start_parallel_update()
    c.runner()
    # while True:
    #     c.read_all_sensors()
    #     c.log_all_sensor()

    c.arduino_reader.join()
    print("Finsh")
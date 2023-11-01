# Import essential libraries
import copy

import requests
import cv2
import numpy as np
import imutils
import time
# https://www.geeksforgeeks.org/connect-your-android-phone-camera-to-opencv-python/
# Replace the below URL with your own. Make sure to add "/shot.jpg" at last.
url = "http://192.168.1.3:8080/shot.jpg"

from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n-pose.pt')  # load an official model


# Train the model
# results = model.train(data='coco8-pose.yaml', epochs=100, imgsz=640)

def get_pose(width=1000, height=1800):
    try:
        start_time = time.time()
        img_resp = requests.get(url)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        img = imutils.resize(img, width=width, height=height)
        # cv2.imshow("Android_cam", img)
        # Predict with the model
        # results = model("http://192.168.1.3:8080/shot.jpg")  # predict on an image
        results = model(img, conf=0.7,
                        max_det=1 # one person at time
                        )  # predict on an image
        result0 = results[0]
        annotated_frame = result0.plot()
        annotated_frame = imutils.resize(annotated_frame, width=width, height=height)

        result0 = result0.numpy()
        head_points = result0.keypoints.data[:, :5]
        head_points_norm = np.copy(head_points)
        head_points_norm[0, :, 0:2] = result0.keypoints.xyn[:, :5]

        condition = head_points_norm[:, :, 2] > 0.5
        average = np.mean(head_points_norm[condition], axis=0)

        end_time = time.time()
        fps = 1 / (end_time - start_time)
        print("FPS :", fps)

        cv2.putText(annotated_frame, "FPS :" + str(int(fps)), (10, 50), cv2.FONT_HERSHEY_COMPLEX, 1.2, (255, 0, 255), 1,
                    cv2.LINE_AA)
        if not np.isnan(average).any():
            x_coord = int(average[0]*annotated_frame.shape[1])
            y_coord = int(average[1]*annotated_frame.shape[1])
            cv2.circle(annotated_frame, (x_coord, y_coord), 5, [0,255,0], -1, lineType=cv2.LINE_AA)
        cv2.imshow("YOLOv8 Inference", annotated_frame)
    except requests.exceptions.ConnectionError as e:
        print(e)
        return float("nan"), float("nan")
    return average[0], average[1]

if __name__ == "__main__":
    # While loop to continuously fetching data from the Url
    while True:
        get_pose()
        # Press Esc key to exit
        if cv2.waitKey(1) == 27:
            break

    cv2.destroyAllWindows()
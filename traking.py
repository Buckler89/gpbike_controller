import cv2
import sys
import time
# Load video
cap = cv2.VideoCapture("C:\\Users\\buckler\Downloads\\traffic_-_27260 (540p).mp4")

# Read first frame
success, frame = cap.read()
if not success:
    print('Failed to read video')
    sys.exit(1)

# Select ROI
bbox = cv2.selectROI(frame, False)
cv2.destroyAllWindows()

# Initialize tracker
# tracker = cv2.TrackerCSRT_create()
# tracker = cv2.TrackerKCF_create()
# tracker = cv2.TrackerBoosting_create()
# tracker = cv2.TrackerMIL_create()
# tracker = cv2.TrackerTLD_create()
tracker = cv2.legacy.TrackerMedianFlow_create()
# tracker = cv2.legacy.TrackerMOSSE_create()


# Function to save cropped image
def save_cropped_image(frame, bbox):
    x, y, w, h = map(int, bbox)
    frame_height, frame_width = frame.shape[:2]

    scale = 2

    # Calculate new dimensions
    crop_w = scale * w
    crop_h = scale * h
    # Adjust top-left corner
    crop_x = max(x - w // scale, 0)
    crop_y = max(y - h // scale, 0)

    new_x = x - crop_x
    new_y = y - crop_y

    # Ensure the new bounding box is within the frame
    crop_x_end = min(crop_x + crop_w, frame_width)
    crop_y_end = min(crop_y + crop_h, frame_height)

    # Crop the new region
    enlarged_bbox = frame[crop_y:crop_y_end, crop_x:crop_x_end]
    crop_bbox = (crop_x, crop_y, crop_w, crop_h)
    new_bbox = (new_x, new_y, w, h)

    frame_debug = cv2.circle(frame, (x, y), radius=0, color=(0, 0, 255), thickness=-1)
    frame_debug = cv2.circle(frame_debug, (new_x, new_y), radius=0, color=(0, 0, 255), thickness=-1)
    cv2.imshow("debug", frame_debug)
    cv2.imshow("debug2", enlarged_bbox)
    return enlarged_bbox, crop_bbox, new_bbox


# frame, crop_bbox, bbox = save_cropped_image(frame, bbox)
cv2.imshow("Tracking", frame)

if cv2.waitKey(1) & 0xFF == 27:  # Esc pressed
    pass
# tracker.init(frame, bbox)
# Process video
while True:
    start_time = time.time()
    success, frame = cap.read()
    frame, crop_bbox, bbox = save_cropped_image(frame, bbox)
    if not success:
        break

    # Update tracker
    # success, bbox = tracker.update(frame)

    # Draw bounding box
    if success:
        (x, y, w, h) = [int(v) for v in bbox]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
        x, y, w, h = map(int, bbox)
    else:
        cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

    # Display result
    cv2.imshow("Tracking", frame)

    end_time = time.time()
    fps = 1 / (end_time - start_time)
    print("FPS :", fps)

    # Exit if ESC pressed
    if cv2.waitKey(1) & 0xFF == 27: # Esc pressed
        break

# Release everything if job is finished
cap.release()
cv2.destroyAllWindows()

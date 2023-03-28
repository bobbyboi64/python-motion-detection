import cv2
import pyvirtualcam
import numpy as np
import keyboard

# Threshold for detecting motion
threshold = 15

# Minimum size of the object to be tracked
min_contour_area = 500

# Color of the bounding box
box_color = (0, 255, 0)

def toggle_toggle():
    global toggle
    toggle = not toggle

toggle = False
camera_list = []
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        camera_name = cap.getBackendName()
        camera_list.append(f"Camera {i}: {camera_name}")
        cap.release()

print("Available cameras:")
for camera in camera_list:
    print(camera)


keyboard.add_hotkey("v", toggle_toggle)

while True:
    camera_choice = input("Choose a camera (0-9): ")
    if camera_choice.isdigit() and int(camera_choice) < len(camera_list):
        camera_choice = int(camera_choice)
        break
    else:
        print("Invalid camera choice.")

cap = cv2.VideoCapture(camera_choice)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

with pyvirtualcam.Camera(width=width, height=height, fps=30) as cam:
    print(f'Virtual camera created: {cam.device}')

    prev_frame = None
    bbox = None

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Compute motion mask
        if prev_frame is not None:
            diff = cv2.absdiff(frame_rgb, prev_frame)
            gray = cv2.cvtColor(diff, cv2.COLOR_RGB2GRAY)
            motion_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)[1]
        else:
            motion_mask = np.zeros((height, width), dtype=np.uint8)

        # Update previous frame
        prev_frame = frame_rgb

        # Apply motion mask to original frame
        frame_rgb = cv2.bitwise_and(frame_rgb, frame_rgb, mask=motion_mask)

        # Find contours of moving objects
        contours, hierarchy = cv2.findContours(motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_contour_area:
                valid_contours.append(contour)

        # Update bounding box if a single object is detected
        if len(valid_contours) == 1:
            x, y, w, h = cv2.boundingRect(valid_contours[0])
            bbox = (x, y, x+w, y+h)
        else:
            bbox = None

        # Draw bounding box
        if bbox is not None:
            cv2.rectangle(frame_rgb, (bbox[0], bbox[1]), (bbox[2], bbox[3]), box_color, thickness=2)

        # Send frame to virtual camera
        if toggle:
            cam.send(framenew)
        else:
            cam.send(frame_rgb)
            framenew = frame_rgb

        cam.sleep_until_next_frame()

cap.release()

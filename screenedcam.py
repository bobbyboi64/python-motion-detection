import cv2
import pyvirtualcam
import numpy as np
import keyboard

# Threshold for detecting motion
threshold = 15

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

    # Previous frame for computing frame difference
    prev_frame = None

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Compute frame difference
        if prev_frame is not None:
            diff = cv2.absdiff(frame_rgb, prev_frame)
            gray = cv2.cvtColor(diff, cv2.COLOR_RGB2GRAY)
            motion_mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)[1]
        else:
            motion_mask = np.zeros((height, width), dtype=np.uint8)

        # Update previous frame
        prev_frame = frame_rgb

        # Display motion mask
        cv2.imshow("Motion mask", motion_mask)

        # Apply motion mask to original frame
        frame_rgb = cv2.bitwise_and(frame_rgb, frame_rgb, mask=motion_mask)

        key = cv2.waitKey(1)
        if toggle:
            cam.send(framenew)
        else:
            cam.send(frame_rgb)
            framenew = frame_rgb

        cam.sleep_until_next_frame()

cap.release()

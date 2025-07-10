import cv2

# Define camera IDs
camera_ids = ["/dev/video0", "/dev/video2", "/dev/video3"]
windows = ["Camera 0", "Camera 2", "Camera 3"]

# Open video streams
caps = [cv2.VideoCapture(cam_id) for cam_id in camera_ids]

# Verify all cameras opened
for i, cap in enumerate(caps):
    if not cap.isOpened():
        print(f"❌ Failed to open {camera_ids[i]}")
    else:
        # Optional: set frame size
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    for i, cap in enumerate(caps):
        ret, frame = cap.read()
        if ret and frame is not None:
            cv2.imshow(windows[i], frame)
        else:
            print(f"⚠️ Failed to read frame from {camera_ids[i]}")

    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
for cap in caps:
    cap.release()
cv2.destroyAllWindows()


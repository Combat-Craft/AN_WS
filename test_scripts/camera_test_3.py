import cv2

# Open camera
cap1 = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)  # Use V4L2 backend for more control

# Set lower resolution and frame rate to reduce bandwidth
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap1.set(cv2.CAP_PROP_FPS, 10)

# Check if camera opened successfully
if not cap1.isOpened():
    print("Error: Cannot open /dev/video2")
    exit()

while True:
    ret1, frame1 = cap1.read()
    if not ret1 or frame1 is None:
        print("Error: Failed to read frame")
        break

    cv2.imshow("camera1", frame1)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cv2.destroyAllWindows()

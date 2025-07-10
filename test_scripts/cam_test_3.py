import cv2

cap1 = cv2.VideoCapture("/dev/media2")
cap2 = cv2.VideoCapture("/dev/media1")

if not cap1.isOpened():
    print("Camera 1 (/dev/video2) failed to open")
if not cap2.isOpened():
    print("Camera 2 (/dev/video1) failed to open")

while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if ret1:
        cv2.imshow("Camera 1", frame1)
    if ret2:
        cv2.imshow("Camera 2", frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap1.release()
cap2.release()
cv2.destroyAllWindows()

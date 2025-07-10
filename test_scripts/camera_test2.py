import cv2

cap1 = cv2.VideoCapture("/dev/video3")  # the other valid one



while True:
	ret1, frame1 = cap1.read()
	combined = frame1
	cv2.imshow("camera1", combined)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
		
cap1.release()

cv2.destroyAllWindows()

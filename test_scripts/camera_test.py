import cv2

for i in range(4):
	source = f'/dev/video{i}'

	cap1 = cv2.VideoCapture(source)  # the other valid one

	while True:
		ret1, frame1 = cap1.read()
		if not ret1: break
		cv2.imshow(f"camera{i}", frame1)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
		
cap1.release()

cv2.destroyAllWindows()
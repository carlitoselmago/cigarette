import time
import cv2


cap = cv2.VideoCapture(0)

cap.set(3,1280)

cap.set(4,1024)

time.sleep(2)

cap.set(15, 0.01)
cap.set(16, 0.01)

while True:

	ret, frame = cap.read()

	cv2.imshow("blo",frame)

	if (cv2.waitKey(1) == 27):
			cv2.destroyAllWindows()
			break
	else:
		pass
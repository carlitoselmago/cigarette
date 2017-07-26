import time
import cv2
import subprocess

cap = cv2.VideoCapture(1)

#cap.set(3,1280)
#cap.set(4,1024)


#subprocess.call(['uvcdynctrl', '-d','video1','-L','/home/haxoorx/code/cigarette/src/uvcdynctrl_configs/logitechDark.gpfl'])



#cap.set(15, -8.0)
#cap.set(15, 0.01)
#cap.set(14, 0.01)
firstRun=False
while True:

	ret, frame = cap.read()

	cv2.imshow("blo",frame)
        if not firstRun:
            print ("trying to set webcam settings")
            #time.sleep(2)
            print subprocess.Popen("uvcdynctrl -d video1 -L /home/haxoorx/code/cigarette/src/uvcdynctrl_configs/logitechDark.gpfl", shell=True, stdout=subprocess.PIPE).stdout.read()
	if (cv2.waitKey(1) == 27):
			cv2.destroyAllWindows()
			break
	else:
		pass
        firstRun=True
        
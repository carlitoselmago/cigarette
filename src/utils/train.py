import sys
import numpy as np
import cv2
import glob

sizeSample=10
samples =  np.empty((0,(sizeSample*sizeSample)))
filename="smokes"


for file in glob.glob('train/*.png'):
	
	im = cv2.imread(file)
	im3 = im.copy()

	gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray,(5,5),0)

	thresh = cv2.adaptiveThreshold(blur,255,1,1,11,2)

	roismall=cv2.resize(thresh, (sizeSample, sizeSample)) 
	cv2.imshow('norm',roismall)

	key = cv2.waitKey(0)

	if key == 27:  # (escape to quit)
		sys.exit()
	else:
		
		sample = roismall.reshape((1,(sizeSample*sizeSample)))
		samples = np.append(samples,sample,0)

	print "training complete for "+file
	

np.savetxt("train/"+str(filename)+'-samples.data',samples)


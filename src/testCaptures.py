from utils.knearest import Knearest
import cv2
import time
import sys
import numpy as np
import glob
#K=Knearest()

def FindSubImage(needle,im2):
	#needle = cv2.imread("assets/imagenes/neddle.png")
	haystack = im2

	result = cv2.matchTemplate(needle,haystack,cv2.TM_CCOEFF_NORMED)
	min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
	print("min_val",min_val,"max_val", max_val)
	
	y,x = np.unravel_index(result.argmax(), result.shape)
	return max_val,x,y
	

def loadNeedles():
	global needles
	
	for i,file in enumerate(glob.glob('utils/train/*.png')):
		needles.append(cv2.imread(file,0))
		th,needles[i] = cv2.threshold(needles[i], 215, 255,cv2.THRESH_BINARY)
		needles[i]=needles[i].astype(np.float32)

def isSmoking(img):
	bestScore=0.0
	bestScoreX=0
	bestScoreY=0
	for needle in needles:

		score,scoreX,scoreY=FindSubImage(needle,img)
		if score>bestScore:
			bestScore=score
			bestScoreX=scoreX
			bestScoreY=scoreY

	return (bestScoreX, bestScoreY) if bestScore>0.8 else []



##############
needles=[]
loadNeedles()
maxValue = 235

# binarize
thresh = 1

for file in glob.glob('captures/*.jpg'):
	"""
	needle=cv2.imread("utils/train/slice1.png",0)

	th,needle = cv2.threshold(needle, 215, 255,cv2.THRESH_BINARY)
	needle=needle.astype(np.float32)
	"""
	imgOriginal=cv2.imread(file)

	img=imgOriginal.copy()
	img=img.astype(np.float32)

	img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	th,img = cv2.threshold(img, maxValue, 255,cv2.THRESH_BINARY)

	#check if has smoke


	#issmoking=FindSubImage(needle,img)
	issmoking=isSmoking(img)
	print(issmoking)
	if len(issmoking)>0:

		cv2.rectangle(imgOriginal,(issmoking[0]+10,issmoking[1]+10),(issmoking[0]+20,issmoking[1]+20), (0,120,145), 5)

	cv2.imshow("bla",imgOriginal)
	cv2.imshow("blo",img)

	key = cv2.waitKey(0)

	if key == 27:  # (escape to quit)
		sys.exit()

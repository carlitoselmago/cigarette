from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

try:
	from pyo import *
except:
	print("couldn't import pyo")
import ctypes
import _ctypes
import pygame
import sys
import cv2
import math
import numpy as np
import time
if sys.hexversion >= 0x03000000:
	import _thread as thread
else:
	import thread


_kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

 # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
_frame_surface = pygame.Surface((_kinect.color_frame_desc.Width, _kinect.color_frame_desc.Height), 0, 32)

# here we will store skeleton data 
_bodies = None

handonmouth=[0,0,0,0,0,0,0,0,0]

w=1080#_kinect.depth_frame_desc.Width
h=1920#_kinect.depth_frame_desc.Height
shape=(w,h,4)
windowName="test"
cv2.namedWindow(windowName, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(windowName,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)


def draw_body_bone(frame,joints, jointPoints, color, joint0, joint1):
	joint0State = joints[joint0].TrackingState;
	joint1State = joints[joint1].TrackingState;

	# both joints are not tracked
	if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
		return

	# both joints are not *really* tracked
	if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
		return

	# ok, at least one is good 
	try:
		start = (int(jointPoints[joint0].x), int(jointPoints[joint0].y))
		end = (int(jointPoints[joint1].x), int(jointPoints[joint1].y))

		cv2.line(frame,(start),(end),color,5)
	except:
		pass
	"""
	try:
		pygame.draw.line(_frame_surface, color, start, end, 8)
	except: # need to catch it due to possible invalid positions (with inf)
		pass
	"""

def getJointPosition(joints, jointPoints, joint):
	joint0State = joints[joint].TrackingState;

	# both joints are not tracked
	if (joint0State == PyKinectV2.TrackingState_NotTracked): 
		return

	# both joints are not *really* tracked
	if (joint0State == PyKinectV2.TrackingState_Inferred):
		return

	# ok, at least one is good
	try:
		return int(jointPoints[joint].x), int(jointPoints[joint].y)
	except:
		return False

def midpoint(p1, p2):
	return int((p1[0]+p2[0])/2), int((p1[1]+p2[1])/2)

def getdistance(p1,p2):
	return int(math.sqrt( (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 ))

def getSubRegion(img,cordinates):
	img = img[cordinates[1] :cordinates[1] +  cordinates[3] , cordinates[0] : cordinates[0] + cordinates[2]]
	
	return img

def ease(o,d):
    #return (o+d)/4
    return d

def handOnMouth(index,frame, joints, jointPoints):
	delaytime=10
	radius=10
	ROI=False
	global handonmouth

	head=getJointPosition(joints, jointPoints, PyKinectV2.JointType_Head)
	neck=getJointPosition(joints, jointPoints, PyKinectV2.JointType_Neck)

	if neck and head:
		mouth=midpoint(head,neck)
		radius=abs(int(head[0]-neck[0]))
	else:
		mouth=False

	if mouth:
		#print(mouth)
		#print(radius)
		radius=getdistance(head,neck)
		radiusExtra=int(radius*2)
		cv2.circle(frame,mouth, radius, (255,0,255), 2)


	lefthand=getJointPosition(joints, jointPoints, PyKinectV2.JointType_HandLeft)
	righthand=getJointPosition(joints, jointPoints, PyKinectV2.JointType_HandRight)

	if lefthand and mouth:
		cv2.circle(frame,lefthand, radius, (255,0,0), 2)
		if getdistance(lefthand,mouth)<(radius):
			#hand on mouth
			handonmouth[index]=delaytime

	if righthand and mouth:
		cv2.circle(frame,righthand, radius, (0,0,255), 2)
		if getdistance(righthand,mouth)<(radius):
			#hand on mouth
			handonmouth[index]=delaytime


	if handonmouth[index]>0:
		cv2.circle(frame,mouth, int(radius/2), (255,255,255), 8)
		#case hand disapears (bug)
		if not lefthand or not righthand:
			handonmouth[index]=delaytime


		ROI=[mouth[0]-radius, mouth[1]-radius, int(radius*2), int(radius*2)]


	delaytime=10
	if handonmouth[index]>0:
		handonmouth[index]-=1

	return ROI or []
	


def getSmokings(img):
	imgOriginal=img
	cv2.imwrite(f"captures/{str(time.time())}.jpg", img)
	img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	# binarize
	thresh = 1
	maxValue = 225
	th,img = cv2.threshold(img, maxValue, 255,cv2.THRESH_BINARY)

	#blobs
	#detector = cv2.SimpleBlobDetector()
	#keypoints = detector.detect(img)
	#img = cv2.drawKeypoints(imgOriginal, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	# Set up the SimpleBlobdetector with default parameters.
	params = cv2.SimpleBlobDetector_Params()

	# Change thresholds
	params.minThreshold = 0;
	params.maxThreshold = 256;

	# Filter by Area.
	params.filterByArea = True
	params.minArea = 10

	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.1

	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.5

	# Filter by Inertia
	params.filterByInertia =True
	params.minInertiaRatio = 0.1

	try:
		detector = cv2.SimpleBlobDetector(params)
	except:
		detector = cv2.SimpleBlobDetector_create(params)
	# Detect blobs.
	reversemask=255-img
	keypoints = detector.detect(reversemask)

	im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	#img=im_with_keypoints
	return im_with_keypoints,keypoints

def draw_body(frame, joints, jointPoints, color):
	# Torso
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);

	# Right Arm    
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_ThumbRight);

	# Left Arm
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);
	draw_body_bone(frame,joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft);


def run():
	
	#SYNTH ::::::::::::::::::::::::::::::::

	# Roland JP-8000 Supersaw emulator.
	freq = 100

	s = Server(duplex=0).boot()

	#1

	lfo2 = Sine(freq=freq).range(0.8, 0.2)
	synths = [SuperSaw(freq=freq, detune=lfo2, mul=0.2)]
	synths[0].mul=0

	#2

	lfo = Sine(freq=freq).range(0.1, 0.1)
	lfoo = Sine(freq=.25, mul=3, add=10)
	#osc = SuperSaw(freq=freq, detune=lfo4, mul=0.2)
	synths.append( Blit(freq=[100, 99.7]*lfoo, harms=lfoo, mul=.3).out())
	synths[1].mul=0


	#3
	"""
	#ind = LinTable([(0,20), (200,5), (1000,2), (8191,1)])
	#m = Metro(4).play()
	#tr = TrigEnv(m, table=ind, dur=4)
	#f = CrossFM(carrier=[250.5,250], ratio=[.2499,.2502], ind1=tr, ind2=tr, mul=.2).out()
	#delay= Delay(f, delay=[.8,.8], feedback=.9, mul=1).out()
	#f.mul=0
	
	#synths.append(f)
	"""

	fxs=[]
	mm = Mixer(outs=2, chnls=len(synths), time=.025)

	for i,synth in enumerate(synths):
		#fxs.append(Freeverb(mm[i], size=.8, damp=.8, mul=.5).out())
		fxs.append(Delay(synths[i], delay=[.8,.8], feedback=.8, mul=1).out())
		mm.addInput(i,synth)
		mm.setAmp(i,1,.5)


	s.start()

	#END SYNTHS ::::::::::::::::::::::::::::::::::::


	global _bodies
	ready=False
	#cap = cv2.VideoCapture(1)
	#cap.set(16, -5)
	# -------- Main Program Loop -----------
	while True:
		
		#get color frame
		if _kinect.has_new_color_frame():

			#ret, frame = cap.read()
			frameOriginal = _kinect.get_last_color_frame()
			frameOriginal=np.array(frameOriginal,dtype=np.uint8).reshape(shape)
			frame=frameOriginal.copy()
			ready=True


		#get skeletons
		if _kinect.has_new_body_frame(): 
			_bodies = _kinect.get_last_body_frame()

		#print skeletons

		if _bodies is not None:
			for i in range(_kinect.max_body_count):
				body = _bodies.bodies[i]
				if not body.is_tracked: 
					continue 

				joints = body.joints 
				# convert joint coordinates to color space 
				joint_points = _kinect.body_joints_to_color_space(joints)
				#draw_body(frame,joints, joint_points, (255,255,255))

				#check hand on mouth
				mouthwithhand=handOnMouth(i,frame,joints, joint_points)


				if len(mouthwithhand)>0:
					#we got a ROI
					#cv2.rectangle(frame,(mouthwithhand[0],mouthwithhand[1]),(mouthwithhand[2],mouthwithhand[3]), (0,120,145), 5)
					roi=getSubRegion(frameOriginal,mouthwithhand)

					imgprocessed,keypoints=getSmokings(roi)
					frame=imgprocessed
					"""
					if len(keypoints)>0:
			
						for i,point in enumerate(keypoints):
							if i<len(synths):
								
								X,Y = point.pt
								synths[i].freq=ease(synths[i].freq,X)
								synths[i].mul=1
						
					else:
						for i,synth in enumerate(synths):
							
							synths[i].mul=0 
						#osc.mul=0
						pass

					frame=roi
					#cv2.imshow("roi",roi)
					"""


		if ready:
			cv2.imshow(windowName,frame)
		if (cv2.waitKey(1) == 27):
			cv2.destroyAllWindows()
			_kinect.close()
			break

run()
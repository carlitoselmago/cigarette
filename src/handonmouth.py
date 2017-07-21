from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys
import cv2
import math
import numpy as np
if sys.hexversion >= 0x03000000:
	import _thread as thread
else:
	import thread


_kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

 # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
_frame_surface = pygame.Surface((_kinect.color_frame_desc.Width, _kinect.color_frame_desc.Height), 0, 32)

# here we will store skeleton data 
_bodies = None

handonmouth=0

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
		position = (int(jointPoints[joint].x), int(jointPoints[joint].y))
		return position
	except:
		return False

def midpoint(p1, p2):
    midpoint= (int((p1[0]+p2[0])/2), int((p1[1]+p2[1])/2))
    return midpoint

def getdistance(p1,p2):
	dist = int(math.sqrt( (p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 ))
	return dist

def handOnMouth(frame, joints, jointPoints):
	delaytime=20
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
			handonmouth=delaytime

	if righthand and mouth:
		cv2.circle(frame,righthand, radius, (0,0,255), 2)
		if getdistance(righthand,mouth)<(radius):
			#hand on mouth
			handonmouth=delaytime

	if handonmouth>0:
		cv2.circle(frame,mouth, int(radius/2), (255,255,255), 8)

	handonmouth-=1


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
	global _bodies
	ready=False
	#cap = cv2.VideoCapture(1)
	#cap.set(16, -5)
	# -------- Main Program Loop -----------
	while True:
		
		#get color frame
		if _kinect.has_new_color_frame():
			
			#ret, frame = cap.read()
			frame = _kinect.get_last_color_frame()
			frame=np.array(frame,dtype=np.uint8).reshape(shape)
			ready=True

		#get skeletons
		if _kinect.has_new_body_frame(): 
			_bodies = _kinect.get_last_body_frame()

		#print skeletons

		if _bodies is not None:
			for i in range(0, _kinect.max_body_count):
				body = _bodies.bodies[i]
				if not body.is_tracked: 
					continue 
				
				joints = body.joints 
				# convert joint coordinates to color space 
				joint_points = _kinect.body_joints_to_color_space(joints)
				#draw_body(frame,joints, joint_points, (255,255,255))

				#check hand on mouth
				handOnMouth(frame,joints, joint_points)



		if ready:
			cv2.imshow(windowName,frame)
		if (cv2.waitKey(1) == 27):
			cv2.destroyAllWindows()
			_kinect.close()
			break

run()
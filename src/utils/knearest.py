# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
import glob
import sys
import os

class Knearest():

	DataFolder="utils/train/"

	def __init__(self):

		self.loadSmokesData()

	def loadSmokesData(self):
		
		filename="smokes"

		#######   training part    ############### 
		samples = np.loadtxt(self.DataFolder+filename+'-samples.data',np.float32)
		responses = [float(i) for i, sample in enumerate(samples)]
		responses=np.asarray(responses,dtype=np.float32)

		responses = responses.reshape((responses.size,1))
		self.modelNum = cv2.ml.KNearest_create()
		self.modelNum.train(samples,cv2.ml.ROW_SAMPLE,responses)

	############################# testing part  #########################

	def reverse(self,text):
		return text if len(text) <= 1 else self.reverse(text[1:]) + text[0]

	def knearest(self,im,mincontour=100,maxcountour=400):

		sizeSample=10

		#im=cv2.imread(image)
		#im = cv2.imread('numtest.png')

		#start_time = time.time()
		#print im.dtype ,"IMAGE TYPE"
		if type is "numeric":
			model=self.modelNum

		if type is "palos":
			model=self.modelPalos

		if type is "cardnumbers":

			model=self.modelcardnumbers

		if type is "alfanumeric":
			return "foo"

		if type is "alfa":
			return "foo"	


		height, width = im.shape[:2]
		#cv2.imshow('imagecrop',im)
		#print height,"height"
		#out = np.zeros(im.shape,np.uint8)
		#gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
		#gray=im
		#ret, thresh=cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
		#thresh = cv2.adaptiveThreshold(gray,255,1,1,11,2)
		#thresh= cv2.convertScaleAbs(im)
		thresh=im
		#thresh=gray
		#thresh=cv2.convertScaleAbs(thresh)
		img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		#_, contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		string=''

		maxdistance=0
		maxdistanceIndex=0
		bestNearestScore=1000000000
		lastContX=99999999999999

		for cnt in contours:
		    
		    countourArea=cv2.contourArea(cnt)
		    if countourArea>mincontour and countourArea<400:
				
				[x,y,w,h] = cv2.boundingRect(cnt)
				
				if x<lastContX:

					print "contour X",x
					if  h>8:

						if len(string)>0:
							
							newDistance=w-x
							if newDistance>maxdistance:
								maxdistance=newDistance
								maxdistanceIndex+=1
						roi = thresh[y:y+h,x:x+w]
						roismall = cv2.resize(roi,(sizeSample,sizeSample))
						#print cv2.countNonZero(roismall),"nonzeros"
						#cv2.imwrite("borrame.png",roismall)
						roismall = roismall.reshape((1,sizeSample*sizeSample))
						roismall = np.float32(roismall)
						k=1
						if type is "cardnumbers":
							k=6
						retval, results, neigh_resp, dists = model.findNearest(roismall, k = k)
						#print "retval, results, neigh_resp, dists ",retval, results, neigh_resp, dists
						
						if type is "cardnumbers":
							if dists[0][0] < bestNearestScore:
								string = str(int((results[0][0])))
								lastX=x
								lastW=w
								bestNearestScore=dists[0][0]
							else:
								pass
						else:
							string += str(int((results[0][0])))
							lastX=x
							lastW=w

					lastContX=x

		
		if type is "numeric" and maxdistance>4:

			print string
			print " maxdistance", maxdistance
			cv2.imwrite(string+".png",im)
			string=string[:-maxdistanceIndex-1]+"."+string[-maxdistanceIndex-1:]

		return string

	def testOcr(self,vision,threshold,file,matchTarget,type,arrayNames): #vision instance
		img = cv2.imread(file)
		start_time = time.time()
		symbolimg=img
		symbolimg = cv2.cvtColor(symbolimg,cv2.COLOR_BGR2GRAY)
		symbolimg = cv2.GaussianBlur(symbolimg,(1,1),2)
		ret, symbolimg = cv2.threshold(symbolimg, threshold, 255, cv2.THRESH_BINARY)

		#cv2.imshow('img',symbolimg)
		text=vision.getText(symbolimg,type,mincontour=150,maxcountour=350)

		img = cv2.imread(file)
		#palosNames=["pica","cor","rombo","trebol"]

		#print palosNames[int(text)]
		return False if text is "" else str(arrayNames[int(text)]) == str(matchTarget)
		

	def testOcrFolderSingleTT(self,vision,folderUri,type,arrayNames,threshold):
		tt=threshold

		matches=0
		count=0

		filesList=[]

		for path, subdirs, files in os.walk(folderUri):
		    for name in files:
		        file= os.path.join(path, name)
		        filesList.append(file)


		for fileuri in filesList:
				if "." in fileuri:
					filename=fileuri.split('\\')[-1]
					matchTarget=filename.split(".")[0]#[:1]

					match=vision.CVocr.testOcr(vision,tt,fileuri,matchTarget,type,arrayNames)

					if match:
						matches+=1
					else:
						print matchTarget+" was not correct",fileuri
					count+=1

		print "::::::::::::::::::::::::::::::::"
		print str(matches)+"/"+str(count)+" matches"
		print "::::::::::::::::::::::::::::::::"
			

	def testOcrFolder(self,vision,folderUri,type,arrayNames):
		bestThreshold=0
		bestScore=0

		filesList=[]

		for path, subdirs, files in os.walk(folderUri):
		    for name in files:
		        file= os.path.join(path, name)
		        filesList.append(file)



		for tt in range(90,255):
			threshold=tt
			count=0
			matches=0
			for fileuri in filesList:
				if "." in fileuri:
					filename=fileuri.split('\\')[-1]
					matchTarget=filename.split(".")[0]#[:1]

					match=vision.CVocr.testOcr(vision,tt,fileuri,matchTarget,type,arrayNames)
					if match:
						matches+=1
					count+=1

			if matches>bestScore:
				bestThreshold=tt
				bestScore=matches
			


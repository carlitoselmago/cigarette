import cv2
import numpy as np;
from pyo import *
from time import sleep



def show_webcam(mirror=False):
    
    #SYNTH ::::::::::::::::::::::::::::::::
    
    # Roland JP-8000 Supersaw emulator.
    freq = 0

    s = Server(duplex=0).boot()

    # Roland JP-8000 Supersaw emulator.
    lfo4 = Sine(freq=freq).range(0.1, 0.75)
    osc4 = SuperSaw(freq=freq, detune=lfo4, mul=0.3).out()
    osc4.volume=-200
    s.start()
    
    #END SYNTH ::::::::::::::::::::::::::::
    
    #CAM ::::::::::::::::::::::::::::::::
    
    cam = cv2.VideoCapture(0)
    
    while True:
        ret_val, img = cam.read()
        if mirror: 
                img = cv2.flip(img, 1)
        
        imgOriginal=img
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                
        # binarize
        thresh = 1
        maxValue = 220
        th,img = cv2.threshold(img, 220, 255,cv2.THRESH_BINARY)
        
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
        params.minArea = 30

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia =True
        params.minInertiaRatio = 0.5

        detector = cv2.SimpleBlobDetector(params)

        # Detect blobs.
        reversemask=255-img
        keypoints = detector.detect(reversemask)
        
        im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        img=im_with_keypoints
        
        if len(keypoints)>0:
            X= keypoints[0].pt[0]
            osc4.freq=X
            osc4.volume=2
        else:
            osc4.freq=0
        
        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    
    #END CAM ::::::::::::::::::::::::::::::::

cv2.destroyAllWindows()



def main():
    #synth()
    show_webcam(mirror=True)
    

if __name__ == '__main__':
        main()
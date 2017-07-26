import cv2
import numpy as np;
from pyo import *
from time import sleep
import math
import time
import subprocess


class cigarette():
    
    device=0
    windowName="cigarette"
    threshold=200
    
    def __init__(self):
        print ("init cigarette")
    

    def main(self,mirror=False):

        #SYNTH ::::::::::::::::::::::::::::::::

        # Roland JP-8000 Supersaw emulator.
        freq = 100

        s = Server(duplex=0)
        #s.setInOutDevice(1)
        s.boot()

        synths=[]

        #1

        lfo2 = Sine(freq=freq).range(0.8, 0.2)
        synths.append( SuperSaw(freq=freq, detune=lfo2, mul=0.2))
        synths[0].mul=0

        #2

        lfo = Sine(freq=freq).range(0.1, 0.1)
        lfoo = Sine(freq=.25, mul=3, add=10)
        #osc = SuperSaw(freq=freq, detune=lfo4, mul=0.2)
        synths.append( Blit(freq=[100, 99.7]*lfoo, harms=lfoo, mul=.3).out())
        synths[1].mul=0


        #3
        """
        ind = LinTable([(0,20), (200,5), (1000,2), (8191,1)])
        m = Metro(4).play()
        tr = TrigEnv(m, table=ind, dur=4)
        f = CrossFM(carrier=[250.5,250], ratio=[.2499,.2502], ind1=tr, ind2=tr, mul=.2).out()
        delay= Delay(f, delay=[.8,.8], feedback=.9, mul=1).out()
        f.mul=0

        synths.append(f)
        """

        fxs=[]
        mm = Mixer(outs=2, chnls=len(synths), time=.025)

        for i,synth in enumerate(synths):
            #fxs.append(Freeverb(mm[i], size=.8, damp=.8, mul=.5).out())
            fxs.append(Delay(synths[i], delay=[.8,.8], feedback=.8, mul=1).out())
            mm.addInput(i,synth)
            mm.setAmp(i,1,.5)


        s.start()

        #END SYNTH ::::::::::::::::::::::::::::

        #CAM ::::::::::::::::::::::::::::::::

        cam = cv2.VideoCapture(self.device)
        ret_val, img = cam.read()
        cv2.imshow(self.windowName, img)
        time.sleep(2)
        print subprocess.Popen("uvcdynctrl -d video"+str(self.device)+" -L /home/haxoorx/code/cigarette/src/uvcdynctrl_configs/logitechDark.gpfl", shell=True, stdout=subprocess.PIPE).stdout.read()
        time.sleep(2)
        while True:
            ret_val, img = cam.read()
            if mirror: 
                    img = cv2.flip(img, 1)

            imgOriginal=img
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # binarize
            thresh = 1
            maxValue = self.threshold
            th,img = cv2.threshold(img, 220, 255,cv2.THRESH_BINARY)

            #blobs
            #detector = cv2.SimpleBlobDetector()
            #keypoints = detector.detect(img)
            #img = cv2.drawKeypoints(imgOriginal, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Set up the SimpleBlobdetector with default parameters.
            params = cv2.SimpleBlobDetector_Params()

            # Change thresholds
            params.minThreshold = 0;
            params.maxThreshold = maxValue;

            # Filter by Area.
            params.filterByArea = True
            params.minArea = 20

            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.3

            # Filter by Convexity
            params.filterByConvexity = True
            params.minConvexity = 0.1

            # Filter by Inertia
            params.filterByInertia =True
            params.minInertiaRatio = 0.1

            #detector = cv2.SimpleBlobDetector(params)
            detector = cv2.SimpleBlobDetector_create(params)

            # Detect blobs.
            reversemask=255-img
            keypoints = detector.detect(reversemask)

            im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            img=im_with_keypoints

            if len(keypoints)>0:

                for i,point in enumerate(keypoints):
                    if i<len(synths):
                        X,Y = point.pt
                        synths[i].freq=X
                        #synths[i].freq=ease(synths[i].freq,X)
                        synths[i].mul=1

            else:
                for i,synth in enumerate(synths):
                    synths[i].mul=0
                #osc.mul=0
                pass


            cv2.imshow(self.windowName, img)
            if cv2.waitKey(1) == 27: 
                break  # esc to quit


        #END CAM ::::::::::::::::::::::::::::::::
        s.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    c=cigarette()
    c.main(mirror=True)
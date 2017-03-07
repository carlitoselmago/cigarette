import threading
import pygame as pg
from time import sleep
from pyo import *
import SimpleCV
from SimpleCV import *



def synth():
    # Roland JP-8000 Supersaw emulator.
    freq = 300

    s = Server(duplex=0).boot()

    # Roland JP-8000 Supersaw emulator.
    lfo4 = Sine(freq=freq).range(0.1, 0.75)
    osc4 = SuperSaw(freq=freq, detune=lfo4, mul=0.3).out()

    s.start()


def videoLoop():
    
    display = SimpleCV.Display()
    cam = SimpleCV.Camera()
    normaldisplay = True
    while display.isNotDone():
            
            k=waitKey(33)
            if k==27:    # Esc key to stop
                break
                ts.stop()
                tv.stop()
                sys.exit()
        

            if display.mouseRight:
                    normaldisplay = not(normaldisplay)
                    print "Display Mode:", "Normal" if normaldisplay else "Segmented" 

            img = cam.getImage().flipHorizontal()
            imgOriginal=img
            img=img.binarize(thresh=220, maxv=255, blocksize=3, p=1).invert()

            #dist = img.colorDistance(SimpleCV.Color.BLACK).dilate(2)
            dist=img
            segmented = dist.stretch(200,255)
            blobs = dist.findBlobs(minsize=10,maxsize=1000)#threshval=-1, minsize=0, maxsize=300)
            if blobs:
                firstBlob= blobs[0]
                print "FUMANDO! "+str(firstBlob.x) 
                blobs.show()
                osc4.freq=firstBlob.x
                #play_tone(stream,frequency=firstBlob.x)
            else:
                pass
                imgOriginal.show()



print "gonna load synth now"




ts= threading.Thread(target=synth)
ts.start()

tv = threading.Thread(target=videoLoop)
tv.start()

sleep(20)
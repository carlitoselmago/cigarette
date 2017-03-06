import threading
import pygame as pg
from time import sleep
from pyo import *


print "gonna load synth now"
 


class Cigarette():
    
    
    
    def __init__(self):
        print "cigarette start"
        
        self.t2 = threading.Thread(target=self.video)
        self.t2.start()
        
        
        self.t1 = threading.Thread(target=self.synth)
        self.t1.start()



    def synth(self):

        # Roland JP-8000 Supersaw emulator.
        freq = 300

        self.s = Server(duplex=2).boot()

        # Roland JP-8000 Supersaw emulator.
        self.lfo4 = Sine(freq=freq).range(0.1, 0.75)
        self.osc4 = SuperSaw(freq=freq, detune=self.lfo4, mul=0.3).out()
        self.s.start()
        sleep(20)

  



    def video(self):
        import SimpleCV


        display = SimpleCV.Display()
        cam = SimpleCV.Camera(0)
        normaldisplay = True

        while display.isNotDone():
                
                for event in pg.event.get():
                    if event.type == pg.KEYDOWN:

                        #si el usuario pulsa la tecla ESC, terminar el programa
                        if event.key == pg.K_ESCAPE:
                            self.t1.stop()
                            self.t2.stop()
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
                    self.osc4.freq=firstBlob.x
                    #play_tone(stream,frequency=firstBlob.x)
                else:
                    pass
                    imgOriginal.show()

                """
                if blobs:
                        #circles = blobs.filter([b.isCircle(0.2) for b in blobs])
                        circles=blobs
                        if circles:
                                try:
                                    img.drawCircle((circles[-1].x, circles[-1].y), circles[-1].radius(),SimpleCV.Color.BLUE,3)
                                except:
                                    pass

                if normaldisplay:
                        img.show()
                else:
                        segmented.show()
         """





cig=Cigarette()
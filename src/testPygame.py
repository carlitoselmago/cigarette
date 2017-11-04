import pygame
import sys
from pygame.locals import *

pygame.init()

filename="images/178x162.jpg"
picture = pygame.image.load(filename)
picture = pygame.transform.scale(picture, (2280, 720))
screen = pygame.display.set_mode((640, 480))
screenMode=0



# Event loop
while 1:
    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit()
        if event.type is KEYDOWN:
            if event.key==K_ESCAPE:
                sys.exit()
            if event.key==K_SPACE:
                if screenMode==0:
                    pygame.display.set_mode((640, 480), FULLSCREEN, 16)
                    screenMode=1
                else:
                    pygame.display.set_mode((640, 480))
                    screenMode=0

    screen.blit(picture, (0, 0))
    pygame.display.flip()

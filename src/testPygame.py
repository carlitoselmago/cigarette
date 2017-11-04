import pygame

from pygame.locals import *

pygame.init()

filename="images/178x162.jpg"
picture = pygame.image.load(filename)
picture = pygame.transform.scale(picture, (2280, 720))
screen = pygame.display.set_mode((640, 480))

# Event loop
while 1:
    for event in pygame.event.get():
        if event.type == QUIT:
            sys.exit()

    screen.blit(picture, (0, 0))
    pygame.display.flip()

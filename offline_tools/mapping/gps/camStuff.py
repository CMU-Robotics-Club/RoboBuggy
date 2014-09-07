import pygame
import pygame.camera
pygame.init()
pygame.camera.init()
camlist = pygame.camera.list_cameras()
print 'avilable Camreas:' + str(camlist)+'\n'
camToUse = "/dev/video1"
cam =pygame.camera.Camera(camToUse,(640,480))
print 'useing'+camToUse+'\n'
cam.start()
img = cam.get_image()
pygame.image.save(img, "image.jpg")

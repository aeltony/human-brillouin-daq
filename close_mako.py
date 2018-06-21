from pymba import *
import time
import cv2
import numpy as np
import math

"""
If app crashes or is not closed the correct way, the CMOS camera will not be released 
properly, which means no other programs can access it.This will cause an error to be thrown.
Just run this application and it will release the camera to allow it to be used elsewhere.
"""


vimba = Vimba()
vimba.startup()
system = vimba.getSystem()
if system.GeVTLIsPresent:
    system.runFeatureCommand("GeVDiscoveryAllOnce")
    time.sleep(0.2)
camera_ids = vimba.getCameraIds()
print camera_ids

camera = vimba.getCamera(camera_ids[0])


vimba.shutdown()
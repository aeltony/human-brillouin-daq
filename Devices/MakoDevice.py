import BrillouinDevice
import time

from pymba import *

import imutils
import cv2
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

import numpy as np
from timeit import default_timer as default_timer   #debugging

from PupilDetection import *


# This is the CMOS camera. Included in this file in the processing class
# is pupil tracking.

class MakoDevice(BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(MakoDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Mako"
        self.camera = None
        self.vimba = Vimba()
        self.set_up()

        self.mako_lock = app.mako_lock
        self.runMode = 0    #0 is free running, 1 is scan

        self.camera.ExposureTimeAbs = 200000    # us??

        self.imageHeight = 1000
        self.imageWidth = 1000
        self.bin_size = 2
        self.camera.Height = self.imageHeight # max: 2048
        self.camera.Width = self.imageWidth # max: 2048
        self.camera.OffsetX = 320
        self.camera.OffsetY = 600

        self.camera.startCapture()
        self.camera.runFeatureCommand('AcquisitionStart')
        self.frame.queueFrameCapture()



    # set up default parameters
    def set_up(self):
        self.vimba.startup()
        system = self.vimba.getSystem()

        if system.GeVTLIsPresent:
            system.runFeatureCommand("GeVDiscoveryAllOnce")
            time.sleep(0.2)
        camera_ids = self.vimba.getCameraIds()

        print("CMOS cameras found: ",camera_ids)
        self.camera = self.vimba.getCamera(camera_ids[0])

        self.camera.openCamera()

        # list camera features
        # cameraFeatureNames = self.camera.getFeatureNames()
        # for name in cameraFeatureNames:
        #     print('Camera feature:', name)

        self.camera.AcquisitionMode = 'Continuous'
        #print("Frame rate limit: ")
        #print(self.camera.AcquisitionFrameRateLimit)
        #print(self.camera.AcquisitionFrameRateAbs)

        self.frame = self.camera.getFrame()
        self.frame.announceFrame()

    def __del__(self):
        return

    def shutdown(self):
        print("[MakoDevice] Closing Device")
        self.camera.runFeatureCommand('AcquisitionStop')
        self.camera.endCapture()
        self.camera.revokeAllFrames()
        self.vimba.shutdown()

    # TODO: in free running mode, don't get new data if there is unprocessed data in queue
    # getData() acquires an image from Andor
    # TODO: copy the data from buffer?
    def getData(self):
        with self.mako_lock:
            self.frame.waitFrameCapture(1000)
            self.frame.queueFrameCapture()
            imgData = self.frame.getBufferByteData()
            image_arr = np.ndarray(buffer = imgData,
                           dtype = np.uint8,
                           shape = (self.frame.height,self.frame.width))
            image_arr = image_arr.reshape((self.frame.height//self.bin_size, self.bin_size, \
                self.frame.width//self.bin_size, self.bin_size)).max(3).max(1)
        # print("[MakoDevice] frame acquired, queue = %d" % self.dataQueue.qsize())
        return image_arr



# This class does the computation for free running mode, mostly displaying
# to the GUI
class MakoFreerun(BrillouinDevice.DeviceProcess):
    updateCMOSImageSig = pyqtSignal('PyQt_PyObject')

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(MakoFreerun, self).__init__(device, stopProcessingEvent, finishedTrigger)

        #self.detectPupil = False
        #self.pupilDetectionSettings = None
        self._pupilRadius = 140 # default value
        self.pupilDetector = PupilDetection()

    @property
    def pupilRadius(self):
        with self.flagLock:
            return self._pupilRadius

    # you can use spectCenter as if it were a class attribute, e.g.
    # pixel = self.spectCenter
    # self.spectCenter = spectrumCenter
    @pupilRadius.setter
    def pupilRadius(self, pupilRad):
        with self.flagLock:
            self._pupilRadius = pupilRad

    # data is an numpy array of type int32
    def doComputation(self, data):
        # print("[MakeFreerun] processing")

        # startTime = default_timer()

        dataOriented = np.flip(data.transpose((1,0)),1)
        # dataOriented = np.data.transpose((1,0))
        # returns a tuple of (image, (centerX, centerY))
        # if no pupil found (centerX, centerY) = (np.nan, np.nan)
        (pupilDetectedImage, center) = self.pupilDetector.DetectPupil(dataOriented, self.pupilRadius)
        image = cv2.cvtColor(pupilDetectedImage, cv2.COLOR_BGR2RGB)
        # print('center = ', center)
        self.updateCMOSImageSig.emit((image, center))
        
        # endTime = default_timer()
        # print("[MakoFreerun/doComputation] t1 = %.3f s" % (endTime - startTime))
        
        return (image, center)
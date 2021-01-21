import Devices.BrillouinDevice
import time

import PySpin

import imutils
import cv2
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

import numpy as np
from timeit import default_timer as default_timer   #debugging

from PupilDetection import *

# Called "Mako" for historical reasons, this is a FLIR camera.
# This is the CMOS camera. Included in this file in the processing class
# is pupil tracking.

class MakoDevice(Devices.BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(MakoDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Mako"
        self.system = None
        self.cam_list = None
        self.camera = None
        self.nodemap = None
        self.nodemap_tldevice = None
        self.system = PySpin.System.GetInstance()
        # Retrieve list of cameras from the system
        self.cam_list = self.system.GetCameras()
        if self.cam_list.GetSize()>0:
            print("[MakoDevice] FLIR camera found")
        self.camera = self.cam_list[0]
        # Retrieve TL device nodemap
        self.nodemap_tldevice = self.camera.GetTLDeviceNodeMap()
        # Initialize camera
        self.camera.Init()
        # Retrieve genicam nodemap
        self.nodemap = self.camera.GetNodeMap()
        self.set_up()
        self.camera.BeginAcquisition()
        self.imageHeight = 2200
        self.imageWidth = 3208
        self.bin_size = 1
        self.mako_lock = app.mako_lock
        self.runMode = 0    #0 is free running, 1 is scan
    
    # set up default parameters
    def set_up(self):
        self.camera.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
        self.camera.ExposureTime.SetValue(200000) # us
        self.camera.AcquisitionFrameRateEnable.SetValue(True)
        self.camera.AcquisitionFrameRate.SetValue(5) # Hz

    def __del__(self):
        return

    def shutdown(self):
        print("[MakoDevice] Closing Device")
        try:
            #self.camera.EndAcquisition()
            del self.nodemap
            #self.camera.DeInit()
            del self.nodemap_tldevice
            del self.camera
            self.cam_list.Clear()
            #self.system.ReleaseInstance()
        except:
            print("[MakoDevice] Not closed properly")

    # TODO: in free running mode, don't get new data if there is unprocessed data in queue
    # getData() acquires an image from Mako
    # TODO: copy the data from buffer?
    def getData(self):
        with self.mako_lock:
            #imgData = np.zeros((3208,2200), dtype=int)
            self.image_result = self.camera.GetNextImage(1000)
            if self.image_result.IsIncomplete():
                print('Image incomplete with image status %d ...' % self.image_result.GetImageStatus())
            else:
                width = self.image_result.GetWidth()
                height = self.image_result.GetHeight()
                #print('Grabbed Image with width = %d, height = %d' % (width, height))
            imgData = self.image_result.GetNDArray()
            image_arr = np.ndarray(buffer = imgData,
                            dtype = np.uint8,
                            shape = (height,width))
            self.image_result.Release()
        return image_arr

    def setExpTime(self, expTime):
        #print('[MakoDevice] setExpTime got called with value=', expTime)
        self.changeSetting(self.mako_lock, lambda:self.camera.ExposureTime.SetValue(expTime*1000))
        print("[MakoDevice] Exposure time set to %.3f ms" % expTime)

    def setFrameRate(self, frameRate):
        #print('[MakoDevice] setFrameRate got called with value=', frameRate)
        self.changeSetting(self.mako_lock, lambda:self.camera.AcquisitionFrameRate.SetValue(frameRate))
        print("[MakoDevice] Frame rate set to %.3f Hz" % frameRate)

# This class does the computation for free running mode, mostly displaying
# to the GUI
class MakoFreerun(Devices.BrillouinDevice.DeviceProcess):
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

    @pupilRadius.setter
    def pupilRadius(self, pupilRad):
        with self.flagLock:
            self._pupilRadius = pupilRad

    # data is an numpy array of type int32
    def doComputation(self, data):
        dataOriented = np.flip(data.transpose((1,0)),1)
        # dataOriented = np.data.transpose((1,0))
        # returns a tuple of (image, (centerX, centerY))
        # if no pupil found (centerX, centerY) = (np.nan, np.nan)
        try:
            (pupilDetectedImage, center) = self.pupilDetector.DetectPupil(dataOriented, self.pupilRadius)
            image = cv2.cvtColor(pupilDetectedImage, cv2.COLOR_BGR2RGB)
            print('center = ', center)
        except:
            image = dataOriented
            #print('Pupil detection failed')
            center = (np.nan, np.nan)
        self.updateCMOSImageSig.emit((image, center))
        
        # endTime = default_timer()
        # print("[MakoFreerun/doComputation] t1 = %.3f s" % (endTime - startTime))
        
        return (image, center)
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
        self.imageHeight = 2000 # Max 2200
        self.imageWidth = 2000 # Max 3208
        self.bin_size = 1
        self.system = PySpin.System.GetInstance()
        # Retrieve list of cameras from the system
        self.cam_list = self.system.GetCameras()
        print('size of cam list =', self.cam_list.GetSize())
        if self.cam_list.GetSize()==1:
            self.camera = self.cam_list[0]
            #gentlNodemap = camera.GetTLDeviceNodeMap()
            try:
                self.camera.Init()
            except PySpin.SpinnakerException as ex:
                print(str(ex))
            #nodeMap = camera.GetNodeMap()
            print('[MakoDevice] FLIR camera initialized')
        else:
            print('[MakoDevice] FLIR camera could not be initialized')
        self.set_up()
        self.camera.BeginAcquisition()
        self.mako_lock = app.mako_lock
        self.runMode = 0    #0 is free running, 1 is scan
    
    # set up default parameters
    def set_up(self):
        try:
            self.camera.Width.SetValue(self.imageWidth)
            self.camera.Height.SetValue(self.imageHeight)
            self.camera.OffsetX.SetValue(int(0.5*(3208 - self.imageWidth)))
            self.camera.OffsetY.SetValue(int(0.5*(2200 - self.imageHeight)))
            self.camera.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
            self.camera.ExposureTime.SetValue(200000) # us
            self.camera.GainAuto.SetValue(PySpin.GainAuto_Off)
            self.camera.Gain.SetValue(0) # dB
            self.camera.AcquisitionFrameRateEnable.SetValue(True)
            self.camera.AcquisitionFrameRate.SetValue(5) # Hz
            self.camera.AcquisitionMode.SetValue(0)
        except PySpin.SpinnakerException as ex:
            print(str(ex))
        print('[MakoDevice] Set-up complete')

    def __del__(self):
        return

    def shutdown(self):
        print("[MakoDevice] Closing Device")
        try:
            self.camera.EndAcquisition()
        except:
            print("[MakoDevice] Could not stop acquisition")
        try:
            self.camera.DeInit()
            self.camera = None
            self.cam_list.Clear()
            self.system.ReleaseInstance()
        except:
            print("[MakoDevice] Not closed properly")

    # getData() acquires an image from Mako
    def getData(self):
        with self.mako_lock:
            #imgData = np.zeros((self.imageWidth,self.imageHeight), dtype=int)
            try:
                self.image_result = self.camera.GetNextImage(1000)
            except:
                print('[MakoDevice] Empty buffer when acquiring image')
                imgData = np.zeros((self.imageWidth,self.imageHeight), dtype=int)
                image_arr = np.ndarray(buffer = imgData,
                                       dtype = np.uint8,
                                       shape = (self.imageHeight,self.imageWidth))
                return image_arr
            if self.image_result.IsIncomplete():
                print('[MakoDevice] Image incomplete with image status %d ...' % self.image_result.GetImageStatus())
                imgData = np.zeros((self.imageWidth,self.imageHeight), dtype=int)
                image_arr = np.ndarray(buffer = imgData,
                                       dtype = np.uint8,
                                       shape = (self.imageHeight,self.imageWidth))
                return image_arr
            else:
                width = self.image_result.GetWidth()
                height = self.image_result.GetHeight()
                #print('[MakoDevice] Grabbed Image with width = %d, height = %d' % (width, height))
            image_arr = np.ndarray(buffer = self.image_result.GetNDArray(),
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
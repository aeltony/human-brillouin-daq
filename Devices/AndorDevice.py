import BrillouinDevice
import time
import numpy as np
import DataFitting
from Andor_DLL_wrap.andor_wrap import *
import imutils
import cv2
from PyQt4 import QtGui,QtCore
from PyQt4.QtCore import pyqtSignal

# This is one of the main devices. It simply acquires a single set of data 
# from the Andor EMCCD when the condition AndorDevice.continueEvent() is 
# set from a managing class. 

class AndorDevice(BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(AndorDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Andor"
        self.cam = Andor()
        self.cam.SetVerbose(False)
        self.set_up()
        self.andor_lock = app.andor_lock
        self.runMode = 0    #0 is free running, 1 is scan

        # buffer for Andor DLL image acquisition
        c_int32_p = POINTER(c_int32)
        self.imageBuffer = np.array([0 for i in range(512*512)])
        self.imageBuffer = self.imageBuffer.astype(np.int32)
        self.imageBufferPointer = self.imageBuffer.ctypes.data_as(c_int32_p)

        self.autoExp = False

    # set up default parameters
    def set_up(self):
        self.cam.SetReadMode(4)
        self.cam.SetAcquisitionMode(1)
        self.cam.SetTriggerMode(0)
        self.cam.SetImage(1,8,1,self.cam.width,1,self.cam.height) #changed 1,4,1
        self.cam.SetShutter(1,1,0,0)
        self.cam.SetExposureTime(.3)
        self.cam.SetTemperature(-120)
        self.cam.SetCoolerMode(1)  # Continuous cooling
        self.cam.GetTemperature()
        self.cam.CoolerON()
        while self.cam.temperature > -20:
            self.cam.GetTemperature()
            print "[AndorDevice] EMCCD cooling down, current T: ",self.cam.temperature,"C"
            time.sleep(1)
        self.cam.SetOutputAmplifier(0)
        self.cam.SetPreAmpGain(2)
        self.cam.SetEMGainMode(2)
        self.cam.SetEMAdvanced(0)
        self.cam.SetEMCCDGain(300)
        self.cam.SetHSSpeed(0,1)    # 5 MHz
        self.cam.SetVSSpeed(1)  # 0.5us
        self.cam.SetADChannel(0)

    def __del__(self):
        return 0

    # getData() acquires an image from Andor
    def getData(self):
        if self.autoExp:
            data = self.getData2()
        else:
            with self.andor_lock:
                self.cam.StartAcquisition()
                self.cam.GetAcquiredData2(self.imageBufferPointer)
            imageSize = self.cam.GetAcquiredDataDim()
            # return a copy of the data, since the buffer is reused for next frame
            data = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
        return data


    def getData2(self):
        print "[Andor] getData2 begin"
        testExpTime = .05
        countsTarget = 15000.
        self.cam.SetExposureTime(testExpTime)
        with self.andor_lock:
            self.cam.StartAcquisition()
            self.cam.GetAcquiredData2(self.imageBufferPointer)
        imageSize = self.cam.GetAcquiredDataDim()
        testImage = np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)
        maxCounts = np.amax(testImage)
        print 'maxCounts =', maxCounts
        adjustedExpTime = countsTarget*testExpTime/maxCounts
        if adjustedExpTime > 2:
            adjustedExpTime = 2
        print 'adjustedExpTime =', adjustedExpTime
        self.cam.SetExposureTime(adjustedExpTime)
        with self.andor_lock:
            self.cam.StartAcquisition()
            self.cam.GetAcquiredData2(self.imageBufferPointer)
        imageSize = self.cam.GetAcquiredDataDim()
        # return a copy of the data, since the buffer is reused for next frame
        return np.array(self.imageBuffer[0:imageSize], copy=True, dtype = np.uint16)


    def getAndorSetting(self, functionHandle, attribute):
        result = 0
        if not self.isRunning():
            with self.andor_lock:
                functionHandle()
                result = self.cam.__dict__[attribute]
        else:
            # pause device acquisition first
            self.pause()
            with self.andor_lock:
                functionHandle()
                result = self.cam.__dict__[attribute]
            self.unpause()
        return result


    def setEMCCDGain(self, gain):
        print "[AndorDevice] EM Gain set to %d" % gain
        self.changeSetting(self.andor_lock, lambda:self.cam.SetEMCCDGain(int(gain)))       

    def getEMCCDGain(self):
        gain = self.getAndorSetting(self.cam.GetEMCCDGain, 'gain')
        print "[AndorDevice] EM Gain is %d" % gain
        return gain    

    def getExposure(self):
        with self.andor_lock:
            return self.cam.exposure

    def setExposure(self, exposureTime):
        # print '[AndorDevice] setExposure got called!'
        self.changeSetting(self.andor_lock, lambda:self.cam.SetExposureTime(exposureTime))
        print "[AndorDevice] Exposure set to %f s" % exposureTime

    def forceSetExposure(self, exposureTime):
        self.cam.SetExposureTime(exposureTime)
        print "[AndorDevice] Exposure set to %f s" % exposureTime

    def setTemperature(self, desiredTemp):
        if (desiredTemp < -80 or desiredTemp > 20):
            print "[AndorDevice/setTemperature] Temperature out of range"
            return
        self.changeSetting(self.andor_lock, lambda:self.cam.SetTemperature(int(desiredTemp)))
        print "[AndorDevice] Temperature set to %d" % desiredTemp

    def getTemperature(self):
        temp = self.getAndorSetting(self.cam.GetTemperature, 'temperature')
        # print "[AndorDevice] Temperature = %f" % temp
        return temp

    def setAutoExp(self, autoExpStatus):
        self.autoExp = autoExpStatus
        print 'autoExpStatus =', autoExpStatus

# This class does the computation for free running mode, mostly displaying
# to the GUI
# It has a handle to the Andor device which has the data queue containing
# raw frames from the camera
class AndorProcessFreerun(BrillouinDevice.DeviceProcess):
    updateBrillouinSeqSig = pyqtSignal('PyQt_PyObject')
    updateSpectrum = pyqtSignal('PyQt_PyObject')
    updateEMCCDImageSig = pyqtSignal('PyQt_PyObject')

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(AndorProcessFreerun, self).__init__(device, stopProcessingEvent, finishedTrigger)

        self._spectCenter = 255 # default value
        self._slineIdx = 32 # default value
        self.cropHeight = 3 # typical: 3
        self.cropWidth = 50 # typical: 50

    @property
    def spectCenter(self):
        with self.flagLock:
            return self._spectCenter

    # you can use spectCenter as if it were a class attribute, e.g.
    # pixel = self.spectCenter
    # self.spectCenter = spectrumCenter
    @spectCenter.setter
    def spectCenter(self, spectrumCenter):
        with self.flagLock:
            self._spectCenter = spectrumCenter

    @property
    def slineIdx(self):
        with self.flagLock:
            return self._slineIdx

    @slineIdx.setter
    def slineIdx(self, slineIndex):
        with self.flagLock:
            self._slineIdx = slineIndex

    # data is an numpy array of type int32
    def doComputation(self, data):
        image_array = data # np.array(data, dtype = np.uint16)

        maximum = image_array.max()
        proper_image = np.reshape(image_array, (-1, 512))   # 512 columns, 128 rows (4x1 binning)
        scaled_image = proper_image*(255.0/maximum)
        scaled_image = scaled_image.astype(int)
        scaled_8bit = np.array(scaled_image, dtype = np.uint8)

        # Find spectral line
        sline_idx = self.slineIdx
        
        if (sline_idx < self.cropHeight):
            loc = self.cropHeight
        elif (sline_idx >= proper_image.shape[0]-self.cropHeight):
            loc = proper_image.shape[0]-self.cropHeight-1
        else:
            loc = sline_idx

        sline = proper_image[sline_idx, :]

        mid = self.spectCenter

        sline_crop = sline[mid-self.cropWidth:mid+self.cropWidth]

        cropped = scaled_8bit[loc-self.cropHeight:loc+self.cropHeight+1, mid-self.cropWidth:mid+self.cropWidth]

        self.image_andor = cropped
        #image = imutils.resize(self.image_andor, width=1024)
        image = cv2.resize(self.image_andor, (0,0), fx=1024/(2*self.cropWidth), fy=170/(2*self.cropHeight + 1), \
            interpolation = cv2.INTER_NEAREST)

        #### Fitting Brillouin spectrum
        interPeakDist, fittedSpect = DataFitting.fitSpectrum(np.copy(sline_crop),1e-4,1e-4)

        # emit signals for GUI to update in real time
        self.updateBrillouinSeqSig.emit(interPeakDist)
        self.updateSpectrum.emit((np.copy(sline_crop), np.copy(fittedSpect)))
        self.updateEMCCDImageSig.emit(np.copy(image))

        # return value is pushed into a Queue, which is collected by the ScanManager
        # for global processing (i.e. Brillouin value segmentation)
        return (proper_image, sline_crop, image)
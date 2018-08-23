import sys
import time

from pymba import *
from my_andor.andor_wrap import *
import zaber.serial as zs


#EMCCD class, where settings can be set
class Andor_Camera(object):
    def __init__(self):
        self.cam = Andor()
        # cam.SetDemoReady()
        self.set_up()

    def set_up(self):
        self.cam.SetReadMode(4)
        self.cam.SetAcquisitionMode(1)
        self.cam.SetTriggerMode(0)
        self.cam.SetImage(1,4,1,self.cam.width,1,self.cam.height)
        self.cam.SetShutter(1,1,0,0)
        self.cam.SetExposureTime(.5)
        self.cam.SetTemperature(-80)
        self.cam.SetCoolerMode(1)

        self.cam.GetTemperature()
        print "camera temperature: ",self.cam.temperature
        #self.cam.CoolerOFF()
        
        while self.cam.temperature > -74:
            self.cam.CoolerON()
            self.cam.GetTemperature()
            print "cooling down right now: ",self.cam.temperature
        
        self.cam.SetOutputAmplifier(0)


        self.cam.GetNumberPreAmpGains()
        print self.cam.noGains

        self.cam.GetPreAmpGain()
        print self.cam.preAmpGain

        self.cam.SetPreAmpGain(2)

        self.cam.SetEMAdvanced(1)
        self.cam.SetEMCCDGain(300)

#598, 564
# class for CMOS camera
class Mako_Camera(object):
    """docstring for ClassName"""
    def __init__(self):
        self.camera = None
        self.vimba = Vimba()
        self.set_up()

    def set_up(self):
        self.vimba.startup()
        system = self.vimba.getSystem()
        if system.GeVTLIsPresent:
            system.runFeatureCommand("GeVDiscoveryAllOnce")
            time.sleep(0.2)
        camera_ids = self.vimba.getCameraIds()

        print "camera_ids: ",camera_ids
        self.camera = self.vimba.getCamera(camera_ids[0])
        self.camera.openCamera()
        self.camera.AcquisitionMode = 'Continuous'



class Motor(object):
    def __init__(self):
        self.port = zs.BinarySerial("COM11", timeout = 20, inter_char_timeout = 0.05)
        self.device = zs.BinaryDevice(self.port, 1)
        self.device.home()
        self.microstep_size = 0.047625

        microstep_cmd = zs.BinaryCommand(1, 37, 64)
        self.device.send(microstep_cmd)

        """
        reply = self.port.read()
        if reply.command_number == 255:
            print("An error occurred in device {}. Error code: {}".format(
                    reply.device_number, reply.data))
        """

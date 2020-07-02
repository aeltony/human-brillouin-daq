from __future__ import division

import BrillouinDevice
import time

from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

import numpy as np
import Queue

import threading
import zaber.serial as zs

# Zaber motor does not need to run in  its own thread, so we don't implement a getData() method
class ZaberDevice(BrillouinDevice.Device):
    # motorMovedSig = pyqtSignal('PyQt_PyObject')

	# This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(ZaberDevice, self).__init__(stop_event)   #runMode=0 default

        self.deviceName = "Zaber"
        self.enqueueData = False
        self.commandQueue = Queue.Queue()
        self.homeLoc = 3000.

        # TODO: choose COM port, throw error if port not found
        self.port = zs.BinarySerial("COM4", timeout = 20, inter_char_timeout = 0.05)
        self.device = zs.BinaryDevice(self.port, 1)
        self.device.home()
        self.microstep_size = 0.047625  #um

        self.device.move_abs(self.homeLoc/self.microstep_size)
        print("[ZaberDevice] Motor homed")

        microstep_cmd = zs.BinaryCommand(1, 37, 64)  #set microstep resolution
        self.device.send(microstep_cmd)
        print("[ZaberDevice] Microstep resolution set to %.6f um" % self.microstep_size)

        speed = 26
        speed_cmd = zs.BinaryCommand(1, 42, int(speed/26*894455))  # 894455 = 26mm/s
        self.device.send(speed_cmd)
        print("[ZaberDevice] Motor speed set to %f mm/s" % speed)

        acceleration_cmd = zs.BinaryCommand(1, 43, 600)
        self.device.send(acceleration_cmd)

        self.ZaberLock = app.ZaberLock

        self.updateLock = threading.Lock()
        self._lastPosition = 0
        self.updatePosition()

    def shutdown(self):
        with self.ZaberLock:
            self.port.close()
        print("[ZaberDevice] Closed")

    # Zaber doesn't do any data acquisition. This method sends out commands from the command queue
    # so they can be called asynchronously. Right now don't have a way to Get data, only set
    # command should be a tuple of ('method name', argument)
    def getData(self):
        try:
            cmd = self.commandQueue.get(block=True, timeout=1.0)
            method = getattr(self, cmd[0])
            if cmd[1] is None:
                method()
            else:
                method(cmd[1])
        except Queue.Empty:
            pass

    def setMotorAsync(self, methodName, arg=[]):
        if (arg == []):
            self.commandQueue.put((methodName, None))
        else:
            self.commandQueue.put((methodName, arg[0]))

    # returns current position of motor, in um
    def updatePosition(self):
        with self.ZaberLock:
            reply = self.device.send(60,0)
        with self.updateLock:
            self._lastPosition = reply.data * self.microstep_size
        
	# returns current position of motor, in um
    def getCurrentPosition(self):
    	with self.updateLock:
            return self._lastPosition

    # moves zaber motor to home position
    def moveHome(self):
        # print('[ZaberDevice] moveHome')
        self.moveAbs(self.homeLoc)
    	# with self.ZaberLock:
     #    	self.device.home()
     #    	loc = self.device.send(60,0)
        # return loc.data * self.microstep_size

    # moves zaber motor, called on by forwards and backwards buttons
    # distance in um
    def moveRelative(self, distance):
    	# print("[ZaberDevice] moveRelative %f um" % distance)
    	with self.ZaberLock:
            self.device.move_rel(distance/self.microstep_size)
        self.updatePosition()

    # moves zaber motor to a set location, called on above
    def moveAbs(self, pos):
    	with self.ZaberLock:
            self.device.move_abs(pos/self.microstep_size)
        self.updatePosition()

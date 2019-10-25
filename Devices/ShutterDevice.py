#import BrillouinDevice

from PyQt4 import QtGui,QtCore
from PyQt4.QtCore import pyqtSignal

from ctypes import *


# ShutterDevice does not need to run in its own thread, so we don't implement a getData() method
# TODO: set the USB code in config file
class ShutterDevice:

	dll = WinDLL("C:\\Program Files\\quad-shutter\\Quad Shutter dll and docs\\x64\\PiUsb")

	usbObjCode = 312	# Objective shutter
	usbRefCode = 314	# Reference shutter

	SAMPLE_STATE = (1, 0)
	REFERENCE_STATE = (0, 1)
	CLOSED_STATE = (0, 0)
	OPEN_STATE = (1, 1)

	def __init__(self, app, state=None):
		self.c2 = c_int()
		self.c4 = c_int()
		self.s2 = c_int()
		self.s4 = c_int()

		#connecting to shutters - 312 (objective) and 314(reference)
		self.usbObj = ShutterDevice.dll.piConnectShutter(byref(self.c2), ShutterDevice.usbObjCode)
		self.usbRef = ShutterDevice.dll.piConnectShutter(byref(self.c4), ShutterDevice.usbRefCode)

		if (state == None):
			state = ShutterDevice.SAMPLE_STATE
		self.setShutterState(state)
		self.state = state

	def shutdown(self):
		ShutterDevice.dll.piDisconnectShutter(self.usbObj)
		ShutterDevice.dll.piDisconnectShutter(self.usbRef)
		print "[ShutterDevice] Closed"
		
	# state is a tuple of (Objective, Reference)
	def setShutterState(self, state):
		ShutterDevice.dll.piSetShutterState(state[0], self.usbObj)
		ShutterDevice.dll.piSetShutterState(state[1], self.usbRef)
		self.state = state
		print "[ShutterDevice] (ObjShutter, RefShutter) = (%d, %d)" % (state[0], state[1])

	def getShutterState(self):
		objState = ShutterDevice.dll.piGetShutterState(byref(self.s2), self.usbObj)
		refState = ShutterDevice.dll.piGetShutterState(byref(self.s4), self.usbRef)
		return (objState, refState)
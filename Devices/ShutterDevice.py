#import BrillouinDevice
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
from ctypes import *

# ShutterDevice does not need to run in its own thread, so we don't implement a getData() method
class ShutterDevice:

	usbObjCode = 384	# Objective shutter
	usbRefCode = 338	# Reference shutter

	SAMPLE_STATE = (1, 0)
	REFERENCE_STATE = (0, 1)
	CLOSED_STATE = (0, 0)
	OPEN_STATE = (1, 1)

	def __init__(self, app, state=None):
		error = c_int()
		self.dll = WinDLL("C:\\Program Files (x86)\\Picard Industries\\USB Quad Shutter\\x64\\PiUsb")

		#connecting to shutters
		self.usbObj = c_long(self.dll.piConnectShutter(byref(error), ShutterDevice.usbObjCode))
		if error.value > 0:
			print('Failed to connect to shutter')
		self.usbRef = c_long(self.dll.piConnectShutter(byref(error), ShutterDevice.usbRefCode))
		if error.value > 0:
			print('Failed to connect to shutter')

		if (state == None):
			state = ShutterDevice.SAMPLE_STATE
		print('Initializing shutter state')
		self.setShutterState(state)
		self.state = state

	def shutdown(self):
		self.dll.piDisconnectShutter(byref(self.usbObj))
		self.dll.piDisconnectShutter(byref(self.usbRef))
		print("[ShutterDevice] Closed")
		
	# state is a tuple of (Objective, Reference)
	def setShutterState(self, state):
		self.dll.piSetShutterState(state[0], byref(self.usbObj))
		self.dll.piSetShutterState(state[1], byref(self.usbRef))
		self.state = state
		print("[ShutterDevice] (ObjShutter, RefShutter) = (%d, %d)" % (state[0], state[1]))

	def getShutterState(self):
		objState = self.dll.piGetShutterState(byref(self.error), byref(self.usbObj))
		refState = self.dll.piGetShutterState(byref(self.error), byref(self.usbRef))
		return (objState, refState)

ERROR_CODE = {
    0: 'PI_NO_ERROR',
    1: 'PI_DEVICE_NOT_FOUND',
    2: 'PI_OBJECT_NOT_FOUND',
	3: 'PI_CANNOT_CREATE_OBJECT'
	}
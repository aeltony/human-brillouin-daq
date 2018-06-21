from ctypes import *
"""
Simple program to connect to shutters and rotate between open and closed.
Documentation for functions written in C located at:
C:\Program Files\quad-shutter\Quad Shutter dll and docs\Doc for DLL usage for shutter
Uses ctypes (python library) to access DLL. Documentation for ctypes at:
https://docs.python.org/2.7/library/ctypes.html


"""

#DLL for C - this is a wrapper to make the C functions available in python
dll = WinDLL("C:\\Program Files\\quad-shutter\\Quad Shutter dll and docs\\x64\\PiUsb")

#some functions in C take in unassigned variables, and the function assigns a value to that variable
c2 = c_int()
c4 = c_int()
s2 = c_int()
s4 = c_int()

#connecting to shutters - 312 (objective) and 314(reference)
usb312 = dll.piConnectShutter(byref(c2), 312)
usb314 = dll.piConnectShutter(byref(c4), 314)

#finds state of shutter 0 = closed and 1 = open 
state = dll.piGetShutterState(byref(s2), usb312)
state = dll.piGetShutterState(byref(s4), usb314)

#everytime function is run it cycles through different states of shutters being open/closed
if s2.value == 0 and s4.value == 0:
	error1 = dll.piSetShutterState(1, usb312)
	print "Open/Closed"
elif s2.value == 1 and s4.value == 0:
	error1 = dll.piSetShutterState(1, usb314)
	print "Open/Open"

elif s2.value ==1 and s4.value == 1:
	error = dll.piSetShutterState(0, usb312)
	print "Closed/Open"
else:
	error = dll.piSetShutterState(0, usb314)
	print "Closed/Closed"

#disconnect from shutters to allow other programs to connect to them.
dll.piDisconnectShutter(usb312)
dll.piDisconnectShutter(usb314)
import Devices.BrillouinDevice
import time
import struct
import serial
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal

# This is the DS Instruments microwave source
# Microwave source does not need to run in  its own thread, so we don't implement a getData() method
class SynthDevice(Devices.BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(SynthDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Synth"
        self.badCommand = b'[BADCOMMAND]\r\n'    # response if a command failed (b makes it into bytes)
        self.port = serial.Serial("COM3", 115200, timeout=10) #Change the COM PORT NUMBER to match your device
        if self.port.isOpen():    # make sure port is open
            self.port.write(b'*IDN?\n')   # send the standard SCPI identify command
            result = self.port.readline()
            print("[SynthDevice] Microwave source found: " + (result.strip()).decode('utf-8'))
        else:
            print('[SynthDevice] Could not open port')
        # Enable RF output
        self.port.write(b'OUTP:STAT ON\n')
        self.port.write(b'OUTP:STAT?\n')
        result = self.port.readline()
        print('[SynthDevice] RF output is ' + (result.strip()).decode('utf-8'))
        # Set initial RF power in dBm
        self.port.write(b'POWER 1\n')
        self.port.write(b'POWER?\n')
        result = self.port.readline()
        print('[SynthDevice] RF power set to ' + (result.strip()).decode('utf-8'))
        # Set initial RF frequency in GHz
        self.port.write(b'FREQ:CW 5GHz\n')
        self.port.write(b'FREQ:CW?\n')
        result = self.port.readline()
        print('[SynthDevice] RF frequency set to ' + (result.strip()).decode('utf-8'))
        self.synth_lock = app.synth_lock
        self.runMode = 0    #0 is free running, 1 is scan

    def shutdown(self):
        with self.synth_lock:
            self.port.write(b'OUTP:STAT OFF\n')
            time.sleep(0.1)
            self.port.close()
        print("[SynthDevice] Closed")

    def __del__(self):
        return 0

    # getData() polls the RF frequency
    def getData(self):
        #with self.synth_lock:
        #    self.port.write(b'FREQ:CW?\n')  # try asking for signal generator setting
        #    result = self.port.readline()
        #    freq = float(result[:-4])*1e-9
        return
    
    def getFreq(self):
        with self.synth_lock:
            self.port.write(b'FREQ:CW?\n')  # try asking for signal generator setting
            result = self.port.readline()
            freq = float(result[:-4])*1e-9
            #print('freq is',freq)
        return freq

    def setFreq(self, freq):
        #print('[SynthDevice] setFreq got called with f =', freq)
        command = 'FREQ:CW %.3f GHz' % freq
        #print('Sent command', command)
        self.changeSetting(self.synth_lock, lambda:self.port.write(command.encode('utf-8')))
        print("[SynthDevice] RF frequency set to %.3f GHz" % freq)

    def getPower(self):
        with self.synth_lock:
            self.port.write(b'POWER?\n')
            result = self.port.readline()
            power = float(result[:-5])
            #print('power is',power)
            return power

    def setPower(self, power):
        #print('[SynthDevice] setPower got called!')
        command = 'POWER %.1f' % power
        self.changeSetting(self.synth_lock, lambda:self.port.write(command.encode('utf-8')))
        print("[SynthDevice] Power set to %.1f dBm" % power)


# This class does the computation for free running mode
class SynthProcessFreerun(Devices.BrillouinDevice.DeviceProcess):

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(SynthProcessFreerun, self).__init__(device, stopProcessingEvent, finishedTrigger)

    def doComputation(self, data):
        # Synth device just returns the RF frequency
        return (data)
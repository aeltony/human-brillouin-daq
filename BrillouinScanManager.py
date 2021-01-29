import threading
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import queue as Queue
from timeit import default_timer as timer   #debugging
from time import sleep
import numpy as np
from ExperimentData import *
import DataFitting

# Scans are sequential measurements comprised of one or many different pieces 
# of hardware. A subset of the data acqusition are taken sequentially while 
# others can be free running in their own threads. The ScanManager controls the 
# sequential data acqusitions and synchronizes with the free running ones using
# ??? (time tags?). Processing of data from individual instruments are done in 
# their corresponding threads asynchronously from both the scan manager and the 
# data acqusition threads. 
# The scan manager also synchronizes the processed data
# from the different processing threads (and does a final processing combining
# different pieces of data?) and sends a signal to the GUI thread for live update.

class ScanManager(QtCore.QThread):
	motorPosUpdateSig = pyqtSignal(list)
	clearGUISig = pyqtSignal()

	#TODO: add a pause event
	def __init__(self, stop_event, motor, shutter):
		super(ScanManager,self).__init__()
        # TODO: change to dictionary
		self.sequentialAcqList = []
		self.sequentialProcessingList = []
		self.freerunningList = []
		self.stop_event = stop_event

		self.motor = motor
		self.shutter = shutter

		self.processor = None

		self.saveScan = True
		self.sessionData = None
		self.saveExpIndex = -1	# this is the expScanIndices argument in the Session.saveToFile() method

		self.scanSettings = None
		#self.pauseEvent = pauseEvent
		self.Cancel_Flag = False


	# TODO: add a lock for accessing these variables
	def assignScanSettings(self, settings):
		# print '[ScanManager/assignScanSettings]'
		self.scanSettings = settings

	# deviceThread is a BrillouinDevice object
	# Make sure only to use thread-safe parts of BrillouinDevice, like
	# setting/clearing events or flags
	def addToSequentialList(self, deviceThread, processingThread):
		self.sequentialAcqList.append(deviceThread)
		self.sequentialProcessingList.append(processingThread)

	def addToFreerunningList(self, deviceThread):
		self.freerunningList.append(deviceThread)

	# processor is a method that takes a list of Brillouin shifts and other arguments as needed
	def addDataProcessor(self, processor):
		self.processor = processor

  	# In a sequential scan, the following sequence must be followed
  	#	dev.doSomethingStart()
  	#	dev.doSomethingWait()
  	#	dev.doSomethingContinue()
  	#
  	#   devThread.continueEvent.set()
    #   devThread.completeEvent.wait()
    #   devThread.completeEvent.clear()
	def run(self):
		self.setPriority(QtCore.QThread.TimeCriticalPriority)

		# make sure saving settings are ok
		if self.saveScan:
			if self.sessionData is None:
				print("No Session provided to save data in; set ScanManager.sessionData first")
				return
			# if self.filename is None:
			# 	print "No data file provided to save; set ScanManager.filename first"
			# 	return
			if self.saveExpIndex == -1:
				print("Save parameter is empty; set ScanManager.saveParameter first")
				return

		# Initialize experiment, like open shutters
		# if (np.abs(self.motor.getCurrentPosition()) > 0.5):	# home motor only if necessary to avoid delay
		# 	self.motor.moveHome()
		# print self.scanSettings['start']
		startPos = self.scanSettings['start']
		self.motor.setMotorAsync('moveRelative', 'x', [startPos[0]])
		self.motor.setMotorAsync('moveRelative', 'y', [startPos[1]])
		self.motor.setMotorAsync('moveRelative', 'z', [startPos[2]])
		# Send motor position signal to update GUI
		motorPos = self.motor.updatePosition()
		self.motorPosUpdateSig.emit(motorPos)
		motorStart = np.copy(motorPos)
		self.shutter.setShutterState((1, 0))

		# print "[ScanManager/run] start"
		print('self.sequentialAcqList =', self.sequentialAcqList)
		print('self.sequentialProcessingList =', self.sequentialProcessingList)

		# first turn off free running mode
		for dev in self.sequentialAcqList:
			dev.sendPauseSignal()
		for dev in self.sequentialAcqList:
			dev.waitForPause()
			dev.runMode = 1

		# free running mode off now; safe to force hardware settings
		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])

		# startTime = timer()


		# Pause all processors and clear any data 
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				sleep(0.1)
			devProcessor.enqueueData = True		
			while not devProcessor.processedData.empty():
				devProcessor.processedData.get()	
			
		# Send signal to clear GUI plots
		self.clearGUISig.emit()

		for dev in self.sequentialAcqList:
			dev.unpause()

		# endTime1 = timer()
		# print "[ScanManager/run] t1 = %.3f s" % (endTime1 - startTime)

		frames = self.scanSettings['frames']
		step = self.scanSettings['step']
		motorCoords = np.empty([1,3])

		

		for i in range(frames[0]):
			# if i > 0:
			# 	self.motor.setMotorAsync('moveAbs', 'y', [motorStart[1]])
			# 	self.motor.setMotorAsync('moveAbs', 'z', [motorStart[2]])
			self.motor.setMotorAsync('moveAbs', 'x', [motorStart[0]+i*step[0]])
			sleep(0.1)
			for j in range(frames[1]):
				# if j > 0:
				# 	self.motor.setMotorAsync('moveAbs', 'z', [motorStart[2]])
				self.motor.setMotorAsync('moveAbs', 'y', [motorStart[1]+j*step[1]])
				sleep(0.1)
				for k in range(frames[2]):
					self.motor.setMotorAsync('moveAbs', 'z', [motorStart[2]+k*step[2]])
					sleep(0.1)
					# Signal all devices to start new acquisition
					for dev in self.sequentialAcqList:
						dev.continueEvent.set()

					# Send motor position signal to update GUI
					motorPos = self.motor.updatePosition()
					dist = np.array(motorPos) - motorStart
					motorCoords = np.vstack((motorCoords,dist))
					self.motorPosUpdateSig.emit(motorPos)

					# synchronization... wait for all the device threads to complete
					for dev in self.sequentialAcqList:
						dev.completeEvent.wait()
						dev.completeEvent.clear()
					
				
				# take calibration data
				self.shutter.setShutterState((0, 1)) # switch to reference arm
				self.sequentialAcqList[0].forceSetExposure(self.scanSettings['refExp'])
				for k in np.arange(5):
					# Signal all devices to start new acquisition
					for dev in self.sequentialAcqList:
						dev.continueEvent.set()
					# synchronization... wait for all the device threads to complete
					for dev in self.sequentialAcqList:
						dev.completeEvent.wait()
						dev.completeEvent.clear()
				# return to sample arm
				self.shutter.setShutterState((1, 0))
				
				self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
				if self.Cancel_Flag == True:
					break
			if self.Cancel_Flag == True:
				# self.sessionData.experimentList[self.saveExpIndex].addScan(volumeScan)
				# scanIdx = self.sessionData.experimentList[self.saveExpIndex].size() - 1
				# self.sessionData.saveToFile([(self.saveExpIndex,[scanIdx])] )

				# # finally return to free running settings before the scan started
				# for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
				# 	devProcessor.enqueueData = False
				# 	dev.runMode = 0
				for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
					devProcessor.enqueueData = False
					dev.runMode = 0
				# Send signal to clear GUI plots 
				self.clearGUISig.emit()
				self.Cancel_Flag = False

				self.maxScanPoints = 400 # Re-scale plot window for free-running mode


				return

		# print "[ScanManager/run] finished"

		# take calibration data
		# self.shutter.setShutterState((0, 1)) # switch to reference arm
		# self.sequentialAcqList[0].forceSetExposure(self.scanSettings['refExp'])
		# for k in np.arange(5):
		# 	# Signal all devices to start new acquisition
		# 	for dev in self.sequentialAcqList:
		# 		dev.continueEvent.set()
		# 		# synchronization... wait for all the device threads to complete
		# 	for dev in self.sequentialAcqList:
		# 		dev.completeEvent.wait()
		# 		dev.completeEvent.clear()
		# # return to sample arm
		# self.shutter.setShutterState((1, 0))
		# self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])

		# return motor to start position
		self.motor.setMotorAsync('moveRelative', 'x', [startPos[0]])
		self.motor.setMotorAsync('moveRelative', 'y', [startPos[1]])
		self.motor.setMotorAsync('moveRelative', 'z', [startPos[2]])
		# Send motor position signal to update GUI
		motorPos = self.motor.updatePosition()
		self.motorPosUpdateSig.emit(motorPos)
		# self.shutter.setShutterState((1, 0))

		# Wait for all processing threads to complete
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				sleep(0.1)

		# Process Data

		#BS = np.random.random()*(self.colormapHigh - self.colormapLow) + self.colormapLow
		dataset = {'Andor': [], 'Mako': [], 'Synth': [], 'TempSensor': []}
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			while devProcessor.processedData.qsize() > frames[0]*frames[1]*frames[2] + 5*frames[0]*frames[1]:		#pop out the first few sets of data stored before scan started
				devProcessor.processedData.get() #pop
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])


		RawTempList = np.array(dataset['TempSensor'])
		print('RawTempList (all list) =', RawTempList)
		CalTempList = np.copy(RawTempList)
		for i in range(5,0,-1):
			RawTempList = np.delete(RawTempList, np.s_[frames[2]::frames[2]+i], 0)
		for i in range(frames[2],0,-1):
			CalTempList = np.delete(CalTempList, np.s_[::i+5], 0)
		waterConst = self.scanSettings['waterConst']
		plasticConst = self.scanSettings['plasticConst']

		pupilCenterList = [d[1] for d in dataset['Mako']]	# list of pupil centre coordinates

		pupilMeanXList = [coord[0] for coord in pupilCenterList]
		pupilMeanYList = [coord[1] for coord in pupilCenterList]

		imageList = [d[0] for d in dataset['Mako']]
		CMOSImage = np.array(imageList)
		for i in range(5,0,-1):
			CMOSImage = np.delete(CMOSImage, np.s_[frames[2]::frames[2]+i], 0)

		specImageList = [d[0] for d in dataset['Andor']]
		EMCCDImage = np.array(specImageList)
		CalImage = np.copy(EMCCDImage)
		for i in range(5,0,-1):
			EMCCDImage = np.delete(EMCCDImage, np.s_[frames[2]::frames[2]+i], 0)
		for i in range(frames[2],0,-1):
			CalImage = np.delete(CalImage, np.s_[::i+5], 0)

		maxRowList = [d[1] for d in dataset['Andor']]
		RawSpecList = np.array(maxRowList)
		CalSpecList = np.copy(RawSpecList)
		for i in range(5,0,-1):
			RawSpecList = np.delete(RawSpecList, np.s_[frames[2]::frames[2]+i], 0)
		for i in range(frames[2],0,-1):
			CalSpecList = np.delete(CalSpecList, np.s_[::i+5], 0)

		dispImageList = [d[2] for d in dataset['Andor']]
		EMCCDDisplay = np.array(dispImageList)

		# Save data
		# volumeScan = ScanData()
		volumeScan = ScanData(timestamp=datetime.now().strftime('%H:%M:%S'))
		# volumeScan.generateTestData(k)
		volumeScan.RawTempList = RawTempList
		volumeScan.CalTempList = CalTempList
		volumeScan.EMCCDImage = EMCCDImage
		volumeScan.CalImage = CalImage
		volumeScan.CMOSImage = CMOSImage
		volumeScan.RawSpecList = RawSpecList
		volumeScan.CalSpecList = CalSpecList
		volumeScan.EMCCDDisplay = EMCCDDisplay
		volumeScan.screenshot = self.scanSettings['screenshot']
		volumeScan.flattenedParamList = self.scanSettings['flattenedParamList']	#save all GUI paramaters

		arr = np.zeros((2,len(pupilMeanXList)))
		arr[0,:] = pupilMeanXList
		arr[1,:] = pupilMeanYList
		volumeScan.PupilDetectorCenters = arr

		if np.all(np.isnan(arr)):
			volumeScan.MeanPupilCenters = np.array([np.nan, np.nan])
		else:
			volumeScan.MeanPupilCenters = np.nanmean(volumeScan.PupilDetectorCenters, 1)
		# print 'volumeScan.MeanPupilCenters =', volumeScan.MeanPupilCenters

		pupilPos = np.transpose(volumeScan.MeanPupilCenters)
		laserPos = np.array([np.float(self.scanSettings['laserX']),np.float(self.scanSettings['laserY'])])
		volumeScan.LaserPos = laserPos

		# measPos = (laserPos - pupilPos)*self.scanSettings['scaleFactor']
		volumeScan.ScanCoords = motorCoords

		startTime = timer()

		#### Fitting Brillouin spectrum
		freqList = np.zeros(RawSpecList.shape[0])
		signal = np.zeros(RawSpecList.shape[0])
		fittedSpect = np.empty(RawSpecList.shape)

		#### Fitting calibration data
		SDcal = np.empty([frames[0]*frames[1]])
		FSRcal = np.empty([frames[0]*frames[1]])
		# Find SD / FSR for every (x, y) coordinate
		for i in range(frames[0]*frames[1]):
			SD = np.array([])
			FSR = np.array([])
			for j in range(5):
				interPeakDist, fittedCalSpect = DataFitting.fitSpectrum(np.copy(CalSpecList[i*5+j]),1e-7,1e-7)
				if len(interPeakDist)==3:
					T = CalTempList[5*i + j]
					WaterBS = waterConst[0]*T*T + waterConst[1]*T + waterConst[2]
					PlasticBS = 16.3291 - (plasticConst[0]*T*T + plasticConst[1]*T + plasticConst[2])
					SDcurr = 2*(PlasticBS - WaterBS)/(interPeakDist[1] + interPeakDist[2])
					FSRcurr = 2*WaterBS + interPeakDist[1]*SD
					SD = np.append(SD, SDcurr)
					FSR = np.append(FSR, FSRcurr)
					# SD = np.append(SD, 2*(9.6051 - 5.1157)/(interPeakDist[1] + interPeakDist[2]))
					# FSR = np.append(FSR, 2*5.1157 + interPeakDist[1]*SD[j])
				else:
					print("[ScanManager/run] Calibration #%d failed." %j)
					SD = np.append(SD, np.nan)
					FSR = np.append(FSR, np.nan)
			try:
				if ~np.isnan(np.nanmean(SD)) and ~np.isnan(np.nanmean(FSR)):
					SDcal[i] = np.nanmean(SD)
					FSRcal[i] = np.nanmean(FSR)
				else:
					print('[ScanManager/run] SD / FSR = NaN')
					SDcal[i] = np.nan
					FSRcal[i] = np.nan
			except:
				print('[ScanManager/run] Calibration failed.')
				SDcal[i] = np.nan
				FSRcal[i] = np.nan

			for j in range(frames[2]):
				sline = np.copy(RawSpecList[i*frames[2]+j])
				sline = np.transpose(sline)
				interPeakDist, fittedSpect[i*frames[2]+j] = DataFitting.fitSpectrum(sline,1e-7,1e-7)
				if len(interPeakDist)==2:
					freqList[i*frames[2]+j] = 0.5*(FSRcal[i] - SDcal[i]*interPeakDist[1])
					signal[i*frames[2]+j] = interPeakDist[0]
				else:
					freqList[i*frames[2]+j] = np.nan
					signal[i*frames[2]+j] = np.nan

		volumeScan.SD = SDcal
		volumeScan.FSR = FSRcal
		volumeScan.BSList = freqList
		volumeScan.FitSpecList = fittedSpect

		# freq4D = np.zeros((frames[0],frames[1],frames[2],4), dtype=np.ubyte)
		# print 'freq4D.shape =', freq4D.shape
		# data3D = freqList.reshape((frames[0],frames[1],frames[2]))
		# data4D = freq4D + data3D[...,np.newaxis]
		# print 'data4D.shape =', data4D.shape
		# print 'data4D = ', data4D

		endTime = timer()
		print("[ScanManager] Fitting time = %.3f s" % (endTime - startTime))
		# print 'Brillouin frequency shift list:', freqList

		self.sessionData.experimentList[self.saveExpIndex].addScan(volumeScan)
		scanIdx = self.sessionData.experimentList[self.saveExpIndex].size() - 1
		self.sessionData.saveToFile([(self.saveExpIndex,[scanIdx])] )

		# finally return to free running settings before the scan started
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			devProcessor.enqueueData = False
			dev.runMode = 0

		# Send signal to clear GUI plots 
		self.clearGUISig.emit()

		# print "[ScanManager/run] finished"
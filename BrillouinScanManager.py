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
	motorPosUpdateSig = pyqtSignal(float)
	clearGUISig = pyqtSignal()

	#TODO: add a pause event
	def __init__(self, cancel_event, stop_event, motor, shutter):
		super(ScanManager,self).__init__()
        # TODO: change to dictionary
		self.sequentialAcqList = []
		self.sequentialProcessingList = []
		self.freerunningList = []
		self.cancel_event = cancel_event
		self.stop_event = stop_event
		self.motor = motor
		self.shutter = shutter
		self.processor = None
		self.saveScan = True
		self.sessionData = None
		self.saveExpIndex = -1	# this is the expScanIndices argument in the Session.saveToFile() method
		self.scanSettings = None
		#self.pauseEvent = pauseEvent

	# TODO: add a lock for accessing these variables
	def assignScanSettings(self, settings):
		# print('[ScanManager/assignScanSettings]')
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
			# 	print("No data file provided to save; set ScanManager.filename first")
			# 	return
			if self.saveExpIndex == -1:
				print("Save parameter is empty; set ScanManager.saveParamter first")
				return   


		# Initialize experiment, like open shutters
		# if (np.abs(self.motor.getCurrentPosition()) > 0.5):	# home motor only if necessary to avoid delay
		# 	self.motor.moveHome()
		# print(self.scanSettings['start'])
		self.motor.setMotorAsync('moveRelative', [self.scanSettings['start']])
		self.shutter.setShutterState((1, 0))#SAMPLE_STATE
		# print("[ScanManager/run] start")

		# first turn off free running mode
		for dev in self.sequentialAcqList:
			dev.sendPauseSignal()
		for dev in self.sequentialAcqList:
			dev.waitForPause()
			dev.runMode = 1

		# free running mode off now. safe to force hardware settings
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
		# print("[ScanManager/run] t1 = %.3f s" % (endTime1 - startTime))

		# sleep(0.2)
		# print("remnant data")
		# print(self.sequentialProcessingList[0].processedData.qsize())
		# print(self.sequentialProcessingList[1].processedData.qsize())
		

		# provide scan settings
		# startTime = timer()

		for k in range(self.scanSettings['frames']):
			# Check if "Cancel" button pressed:
			# print('self.cancel_event =', self.cancel_event.is_set())
			if self.cancel_event.is_set():
				# If scan cancelled, skip to calibration data (wrap-up scan)
				break
			else:
				self.motor.setMotorAsync('moveRelative', [self.scanSettings['step']])

				# Signal all devices to start new acquisition
				for dev in self.sequentialAcqList:
					dev.continueEvent.set()

				# Send motor position signal to update GUI
				motorPos = self.motor.getCurrentPosition()
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


		# endTime = timer()
		# print("[ScanManager/run] Scan time = %.3f s" % (endTime - startTime))

		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
		self.motor.moveHome()
		# self.shutter.setShutterState((1, 0))

		# Wait for all processing threads to complete
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				sleep(0.1)

		# Process Data

		#BS = np.random.random()*(self.colormapHigh - self.colormapLow) + self.colormapLow
		dataset = {'Andor': [], 'Mako': [], 'TempSensor': []}
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			while devProcessor.processedData.qsize() > self.scanSettings['frames'] + 5:		#pop out the first few sets of data stored before scan started
				devProcessor.processedData.get() #pop
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])

		RawTempList = np.array(dataset['TempSensor'])[:-5]
		CalTempList = np.array(dataset['TempSensor'])[-5:]
		waterConst = self.scanSettings['waterConst']
		plasticConst = self.scanSettings['plasticConst']

		pupilCenterList = [d[1] for d in dataset['Mako']]	# list of pupil centre coordinates

		pupilMeanXList = [coord[0] for coord in pupilCenterList]
		pupilMeanYList = [coord[1] for coord in pupilCenterList]

		imageList = [d[0] for d in dataset['Mako']]
		CMOSImage = np.array(imageList)[:-5]

		specImageList = [d[0] for d in dataset['Andor']]
		SampleImage = np.array(specImageList)[:-5]
		CalImage = np.array(specImageList)[-5:]

		maxRowList = [d[1] for d in dataset['Andor']]
		RawSpecList = np.array(maxRowList)[:-5]
		CalSpecList = np.array(maxRowList)[-5:]

		dispImageList = [d[2] for d in dataset['Andor']]
		SampleDisplay = np.array(dispImageList)

		expTimeList = [d[3] for d in dataset['Andor']]
		RawTimeList = np.array(expTimeList)[:-5]
		CalTimeList = np.array(expTimeList)[-5:]

		# Save data
		# lineScan = ScanData()
		lineScan = ScanData(timestamp=datetime.now().strftime('%H:%M:%S'))

		lineScan.RawTempList = RawTempList
		lineScan.CalTempList = CalTempList
		# lineScan.generateTestData(k)
		lineScan.SampleImage = SampleImage
		lineScan.CalImage = CalImage
		lineScan.CMOSImage = CMOSImage
		lineScan.RawSpecList = RawSpecList
		lineScan.CalSpecList = CalSpecList
		lineScan.SampleDisplay = SampleDisplay
		lineScan.RawTimeList = RawTimeList
		lineScan.CalTimeList = CalTimeList
		lineScan.screenshot = self.scanSettings['screenshot']
		lineScan.flattenedParamList = self.scanSettings['flattenedParamList']	#save all GUI paramaters

		arr = np.zeros((2,len(pupilMeanXList)))
		arr[0,:] = pupilMeanXList
		arr[1,:] = pupilMeanYList
		lineScan.PupilDetectorCenters = arr

		if np.all(np.isnan(arr)):
			lineScan.MeanPupilCenters = np.array([np.nan, np.nan])
		else:
			lineScan.MeanPupilCenters = np.nanmean(lineScan.PupilDetectorCenters, 1)
		# print('lineScan.MeanPupilCenters =', lineScan.MeanPupilCenters)

		pupilPos = np.transpose(lineScan.MeanPupilCenters)
		laserPos = np.array([np.float(self.scanSettings['laserX']),np.float(self.scanSettings['laserY'])])
		lineScan.LaserPos = laserPos

		measPos = (laserPos - pupilPos)*self.scanSettings['scaleFactor']
		lineScan.ScanCoords = measPos

		#### Fitting calibration data
		SD = np.array([])
		FSR = np.array([])
		for j in range(CalSpecList.shape[0]):
			interPeakDist, fittedSpect = DataFitting.fitSpectrum(np.copy(CalSpecList[j])//CalTimeList[j],1e-7,1e-7)
			if len(interPeakDist)==3:
				T = CalTempList[j]
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
				SDcal = np.nanmean(SD)
				FSRcal = np.nanmean(FSR)
			else:
				print('[ScanManager/run] SD / FSR = NaN')
				SDcal = np.nan
				FSRcal = np.nan
		except:
			print('[ScanManager/run] Calibration failed.')
			SDcal = np.nan
			FSRcal = np.nan

		lineScan.SD = SDcal
		lineScan.FSR = FSRcal

		#### Fitting Brillouin spectrum
		aline = np.zeros(RawSpecList.shape[0])
		signal = np.zeros(RawSpecList.shape[0])
		fittedSpect = np.zeros(RawSpecList.shape)

		for j in range(RawSpecList.shape[0]):
			interPeakDist, fittedSpect[j] = DataFitting.fitSpectrum(np.copy(RawSpecList[j])//RawTimeList[j],1e-7,1e-7)
			if len(interPeakDist)==2:
				aline[j] = 0.5*(FSRcal - SDcal*interPeakDist[1])
				signal[j] = interPeakDist[0]
			else:
				aline[j] = np.nan
				signal[j] = np.nan

		lineScan.BSList = aline
		lineScan.FitSpecList = fittedSpect
		# print('A-line scan:', aline)

		#### Segmentation of A-line scan into air / cornea / aq humor
		steps = np.linspace(0,len(aline)-1,len(aline))
		(lineBS, strIdx) = DataFitting.fitAline(steps, aline, signal)
		# print('lineBS =', lineBS)
		lineScan.BS = lineBS
		lineScan.StromaIdx = strIdx # Indices for segmented region

		self.sessionData.experimentList[self.saveExpIndex].addScan(lineScan)
		scanIdx = self.sessionData.experimentList[self.saveExpIndex].size() - 1
		self.sessionData.saveToFile([(self.saveExpIndex,[scanIdx])] )

		# finally return to free running settings before the scan started
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			devProcessor.enqueueData = False
			dev.runMode = 0

		# Send signal to clear GUI plots 
		self.clearGUISig.emit()

		# print("[ScanManager/run] finished")
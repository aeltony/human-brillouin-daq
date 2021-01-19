import threading
import datetime
import numpy as np
import time
import math
import sys
import os
import ntpath
from scipy import interpolate
from scipy.interpolate import griddata
import csv
from PyQt5 import QtGui, QtCore
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType
from pyqtgraphCustomize import *
import qt_ui # UI import
from ctypes import *
from configparser import ConfigParser
from Devices.SynthDevice import SynthDevice, SynthProcessFreerun
from Devices.AndorDevice import AndorDevice, AndorProcessFreerun
from Devices.MakoDevice import MakoDevice, MakoFreerun
from Devices.TempSensorDevice import TempSensorDevice, TempSensorFreerun
from Devices.ZaberDevice import ZaberDevice
from Devices.ShutterDevice import ShutterDevice
from BrillouinScanManager import ScanManager
from ExperimentData import *
from BrillouinTreeModel import *

class CustomViewBox(pg.ViewBox):
    def __init__(self, *args, **kwds):
        pg.ViewBox.__init__(self, *args, **kwds)
        self.setMouseMode(self.RectMode)
        self.setBackgroundColor((255,255,255,0))
        
    ## reimplement right-click to zoom out
    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.RightButton:
            self.autoRange(padding=0)
            
    def mouseDragEvent(self, ev):
        if ev.button() == QtCore.Qt.RightButton:
            ev.ignore()
        else:
            pg.ViewBox.mouseDragEvent(self, ev)


class App(QtGui.QMainWindow,qt_ui.Ui_MainWindow):
 
    def __init__(self):
        super(App,self).__init__()
        self.setupUi(self)

        # load default/save parameter first
        self.configFilename = 'BrillouinScanConfig.ini'
        self.configParser = ConfigParser()
        self.configParser.read(self.configFilename)
        # TODO: validate configuration file, set defaults if configuration file is corrupted or unavailable
        self.waterConst = np.empty(3)
        self.plasticConst = np.empty(3)
        self.waterConst[0] = self.configParser.getfloat('Calibration constants', 'w1')
        self.waterConst[1] = self.configParser.getfloat('Calibration constants', 'w2')
        self.waterConst[2] = self.configParser.getfloat('Calibration constants', 'w3')
        self.plasticConst[0] = self.configParser.getfloat('Calibration constants', 'p1')
        self.plasticConst[1] = self.configParser.getfloat('Calibration constants', 'p2')
        self.plasticConst[2] = self.configParser.getfloat('Calibration constants', 'p3')
        laserX = self.configParser.getint('Scan', 'laser_position_X')
        laserY = self.configParser.getint('Scan', 'laser_position_Y')
        RFpower = self.configParser.getfloat('Synth', 'RFpower')
        sampleSpectCenter = self.configParser.getint('Andor', 'sampleSpectCenter')
        calibSpectCenter = self.configParser.getint('Andor', 'calibSpectCenter')
        sampleSlineIdx = self.configParser.getint('Andor', 'sampleSlineIdx')
        calibSlineIdx = self.configParser.getint('Andor', 'calibSlineIdx')
        pupilRadius = self.configParser.getfloat('Mako', 'pupilRadius')

        self.params = [
            {'name': 'Scan', 'type': 'group', 'children': [
                {'name': 'Start Position', 'type': 'group', 'children': [
                    {'name': 'X', 'type': 'float', 'value': 0, 'suffix':' um', 'step': 100, 'limits':(-10000,10000),'decimals':5},
                    {'name': 'Y', 'type': 'float', 'value': 0, 'suffix':' um', 'step': 100, 'limits':(-10000,10000),'decimals':5},
                    {'name': 'Z', 'type': 'float', 'value': 0, 'suffix':' um', 'step': 100, 'limits':(-10000,10000),'decimals':5}]},
                {'name': 'Step Size', 'type': 'group', 'children': [
                    {'name': 'X', 'type': 'float', 'value': 50, 'suffix':' um', 'step': 1, 'limits':(0,1000),'decimals':5},
                    {'name': 'Y', 'type': 'float', 'value': 50, 'suffix':' um', 'step': 1, 'limits':(0,1000),'decimals':5},
                    {'name': 'Z', 'type': 'float', 'value': 50, 'suffix':' um', 'step': 1, 'limits':(0,1000),'decimals':5}]},
                {'name': 'Frame Number', 'type': 'group', 'children': [
                    {'name': 'X', 'type': 'int', 'value': 5, 'step': 1, 'limits':(1,2000)},
                    {'name': 'Y', 'type': 'int', 'value': 5, 'step': 1, 'limits':(1,2000)},
                    {'name': 'Z', 'type': 'int', 'value': 5, 'step': 1, 'limits':(1,2000)}]},
                {'name': 'End Position', 'type': 'group', 'children': [
                    {'name': 'X', 'type': 'float', 'value': 250, 'suffix':' um', 'readonly': True, 'decimals':5},
                    {'name': 'Y', 'type': 'float', 'value': 250, 'suffix':' um', 'readonly': True, 'decimals':5},
                    {'name': 'Z', 'type': 'float', 'value': 250, 'suffix':' um', 'readonly': True, 'decimals':5}]},
                {'name': 'Ambient Temp.', 'type': 'float', 'value': 0.0, 'suffix':' deg. C', 'readonly':True, 'decimals':4},
                {'name': 'Ref. FSR', 'type': 'float', 'value':16.25, 'suffix':' GHz', 'limits':(5, 100), 'decimals':6}, 
                {'name': 'Ref. SD', 'type': 'float', 'value':0.1, 'suffix':' GHz/px', 'limits':(0, 2), 'decimals':4}, 
                {'name': 'ToggleReference', 'type':'toggle', 'ButtonText':('Switch to Reference', 'Switch to Sample')},         #False=Sampe="Switch to Ref"
                {'name': 'Scan/Cancel', 'type': 'action2', 'ButtonText':('Scan', 'Cancel')},
                # {'name': 'Scan', 'type':'action'},
                # {'name': 'Pause', 'type':'action'},
                {'name': 'More Settings', 'type': 'group', 'children': [
                    {'name': 'Laser Focus X', 'type': 'int', 'value': laserX, 'suffix':' px', 'limits':(1,2048),'decimals':4},
                    {'name': 'Laser Focus Y', 'type': 'int', 'value': laserY, 'suffix':' px', 'limits':(1,2048),'decimals':4}
                ]}
            ]},
            {'name': 'Motor', 'type': 'group', 'children': [
                {'name': 'Velocity', 'type': 'float', 'value': 26, 'suffix':' (mm/s)', 'step': 1, 'limits': (1, 26)},
                {'name': 'Acceleration', 'type': 'float', 'value': 600, 'suffix':' (mm/s^2)', 'step': 10, 'limits': (1, 1000), 'decimals':4},
                {'name': 'Jog step', 'type': 'float', 'value': 10, 'suffix':' um', 'step': 1, 'limits':(0.1, 500)},
                {'name': 'Current X location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':5},
                {'name': 'Current Y location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':5},
                {'name': 'Current Z location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':5},
                {'name': 'Jog X', 'type': 'action3', 'ButtonText':('Jog X +', 'Jog X -', 'Home X')},
                {'name': 'Jog Y', 'type': 'action3', 'ButtonText':('Jog Y +', 'Jog Y -', 'Home Y')},
                {'name': 'Jog Z', 'type': 'action3', 'ButtonText':('Jog Z +', 'Jog Z -', 'Home Z')},
                {'name': 'Move to location', 'type': 'float', 'value':0, 'suffix':' um', 'limits':(0, 5000), 'decimals':5},
                {'name': 'Move', 'type':'action3', 'ButtonText':('Move X', 'Move Y', 'Move Z')}
            ]},
            {'name': 'Microwave Source', 'type': 'group', 'children': [
                {'name': 'RF Frequency', 'type': 'float', 'value': 5.0, 'suffix':' GHz', 'step': 0.1, 'limits': (0.05, 13.0), 'decimals':3},
                {'name': 'RF Power', 'type': 'float', 'value': 1.0, 'suffix':' dBm', 'step': 0.5, 'limits': (-20, 10), 'decimals':1}
             ]},
            {'name': 'Spectrometer Camera', 'type': 'group', 'children': [
                {'name': 'AutoExposure', 'type':'toggle', 'ButtonText':('Auto exposure', 'Fixed exposure')},         #False=Fixed exposure
                {'name': 'Camera Temp.', 'type': 'float', 'value':0, 'suffix':' C', 'readonly': True},
                {'name': 'Exposure', 'type':'float', 'value':0.3, 'suffix':' s', 'step':0.05, 'limits':(0.01, 10)},
                {'name': 'Ref. Exposure', 'type':'float', 'value':1.0, 'suffix':' s', 'step':0.05, 'limits':(0.01, 10)},
                {'name': 'Sample Column', 'type':'int', 'value': sampleSpectCenter, 'suffix':' px', 'step':1, 'limits':(0, 2048)},
                {'name': 'Ref. Column', 'type':'int', 'value': calibSpectCenter, 'suffix':' px', 'step':1, 'limits':(0, 2048)},
                {'name': 'Sample Row', 'type':'int', 'value': sampleSlineIdx, 'suffix':' px', 'step':1, 'limits':(0, 2048)},
                {'name': 'Ref. Row', 'type':'int', 'value': calibSlineIdx, 'suffix':' px', 'step':1, 'limits':(0, 2048)},
            ]},        
            {'name': 'Pupil Camera', 'type': 'group', 'children': [
                {'name': 'Pupil Radius', 'type': 'float', 'value': pupilRadius, 'suffix':' px', 'step': 5, 'limits': (1, 1000)},
                {'name': 'Scale Factor', 'type': 'float', 'value': 0.02, 'suffix':' (mm/px)', 'step': 0.001, 'limits': (0, 1)},
                {'name': 'Frame Rate', 'type': 'int', 'value': 5, 'limits':(2, 20)}
                # {'name': 'Take Picture', 'type': 'float', 'value': -60, 'step': 1, 'limits': (-80, 30)},
                # {'name': 'Take Video', 'type': 'float', 'value':0, 'readonly': True},
                #Other settings here
            ]}
        ]
        ## Create tree of Parameter objects
        self.allParameters = Parameter.create(name='params', type='group', children=self.params)
        self.parameterTreeWidget.setParameters(self.allParameters, showTop=False)

        # Laser crosshair adjustment
        self.allParameters.child('Scan').child('More Settings').child('Laser Focus X').sigValueChanging.connect(
            self.CMOSvLineValueChange)
        self.allParameters.child('Scan').child('More Settings').child('Laser Focus Y').sigValueChanging.connect(
            self.CMOShLineValueChange)
        self.allParameters.child('Spectrometer Camera').child('Sample Column').sigValueChanged.connect(
            self.sampleSpectCenterValueChange)
        self.allParameters.child('Spectrometer Camera').child('Ref. Column').sigValueChanged.connect(
            self.calibSpectCenterValueChange)
        self.allParameters.child('Spectrometer Camera').child('Sample Row').sigValueChanged.connect(
            self.sampleSlineIdxValueChange)
        self.allParameters.child('Spectrometer Camera').child('Ref. Row').sigValueChanged.connect(
            self.calibSlineIdxValueChange)
        self.allParameters.child('Pupil Camera').child('Pupil Radius').sigValueChanged.connect(
            self.pupilRadiusValueChange)

        self.model = BrillouinTreeModel()
        self.model.activeExperimentSig.connect(self.updateHeatmapByExp)

        self.treeView.setModel(self.model)
        self.treeView.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self.createTreeMenu)
        self.treeView.setEnabled(False)
        self.treeView.selectionModel().selectionChanged.connect(self.treeSelectionChanged)

        self.session = None
        self.dataFile = None
        self.dataFileName = None
        # self.activeExperiment = None

        self.dataViewerTab.currentChanged.connect(self.dataViewerTabChanged)

        #Lock used to halt other threads upon app closing
        self.stop_event = threading.Event()
        self.cancel_event = threading.Event()
        self.synth_lock = threading.Lock()
        self.andor_lock = threading.Lock()
        self.mako_lock = threading.Lock()
        self.TempSensor_lock = threading.Lock()
        self.ZaberLock = threading.Lock()
        self.scan_lock = threading.Lock()
        self.map_lock = threading.Lock()

        # Even though ZaberDevice is a QThread, for now we don't need to run it in a separate thread
        # It is implemented this way for future-proofing
        self.ZaberDevice = ZaberDevice(self.stop_event, self)
        self.ZaberDevice.start()

        self.TempSensorDeviceThread = TempSensorDevice(self.stop_event, self)
        self.TempSensorProcessThread = TempSensorFreerun(self.TempSensorDeviceThread, self.stop_event)
        self.TempSensorProcessThread.updateTempSeqSig.connect(self.UpdateAmbientTemp)
        self.TempSensorDeviceThread.start()
        self.TempSensorProcessThread.start()

        self.SynthDeviceThread = SynthDevice(self.stop_event, self)
        self.SynthProcessThread = SynthProcessFreerun(self.SynthDeviceThread, self.stop_event)
        self.SynthDeviceThread.start()
        self.SynthProcessThread.start()

        self.ShutterDevice = ShutterDevice(self)    # Initialized to SAMPLE_STATE

        self.MakoDeviceThread = MakoDevice(self.stop_event, self)
        self.MakoProcessThread = MakoFreerun(self.MakoDeviceThread, self.stop_event)
        self.MakoProcessThread.updateCMOSImageSig.connect(self.MakoProcessUpdate)
        self.MakoDeviceThread.start()
        self.MakoProcessThread.start()

        self.AndorDeviceThread = AndorDevice(self.stop_event, self)
        self.AndorProcessThread = AndorProcessFreerun(self.AndorDeviceThread, self.stop_event)
        self.AndorProcessThread.updateSampleImageSig.connect(self.AndorSampleProcessUpdate)
        self.AndorProcessThread.updateCalibImageSig.connect(self.AndorCalibProcessUpdate)

        # self.CMOSview = CustomViewBox(invertY=True)             # The ViewBox is a zoomable/pannable box
        self.CMOSview = pg.PlotItem()
        self.graphicsViewCMOS.setCentralItem(self.CMOSview)     # GraphicsView is the main graphics container
        self.CMOSview.setAspectLocked(True)
        self.CMOSScatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(color='r'))
        self.CMOSImage = pg.ImageItem(np.zeros((self.MakoDeviceThread.imageWidth//self.MakoDeviceThread.bin_size, self.MakoDeviceThread.imageHeight//self.MakoDeviceThread.bin_size)))      # ImageItem contains what needs to be plotted
        self.CMOSview.addItem(self.CMOSImage)
        self.CMOSview.autoRange(padding=0)                      # gets rid of padding
        self.CMOSvLine = pg.InfiniteLine(angle=90, movable=False)
        self.CMOShLine = pg.InfiniteLine(angle=0, movable=False)
        self.CMOSview.addItem(self.CMOSvLine, ignoreBounds=True)
        self.CMOSview.addItem(self.CMOShLine, ignoreBounds=True)
        self.CMOSvLine.setPos(laserX)
        self.CMOShLine.setPos(laserY)
        self.CMOSview.addItem(self.CMOSScatter)
        self.makoPoints = np.array([])    # To display previous scan points

        self.CMOSview_recording = pg.PlotItem()
        self.graphicsViewCMOSViewer.setCentralItem(self.CMOSview_recording)     # GraphicsView is the main graphics container
        self.CMOSview_recording.setAspectLocked(True)
        self.CMOSScatter_recording = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(color='r'))
        self.CMOSImage_recording = pg.ImageItem(np.zeros((self.MakoDeviceThread.imageWidth//self.MakoDeviceThread.bin_size, self.MakoDeviceThread.imageHeight//self.MakoDeviceThread.bin_size)))      # ImageItem contains what needs to be plotted
        self.CMOSview_recording.addItem(self.CMOSImage_recording)
        self.CMOSview_recording.addItem(self.CMOSScatter_recording)
        self.CMOSview_recording.autoRange(padding=0)  # gets rid of padding


        # TODO: put filename for colormap into config file
        arr = []
        with open('Utilities\\colormap.txt', 'r') as f:
            reader = csv.reader(f, delimiter='\t')
            for row in reader:
                arr.append([float(x) for x in row] + [1]) # RGBA, 0.0 to 1.0
        self.heatmapColormapArray = np.array(arr)
        self.colormapLow = 5.58     #TODO: put these in settings
        self.colormapHigh = 5.82
        self.colormap = pg.ColorMap(np.linspace(0, 1, len(self.heatmapColormapArray)), self.heatmapColormapArray)

        # Data acquisition Heatmap
        M = self.MakoDeviceThread.imageWidth
        N = self.MakoDeviceThread.imageHeight
        self.gridx, self.gridy = np.mgrid[-0.5*M:0.5*M:(M+1)*1j, -0.5*N:0.5*N:(N+1)*1j]
        self.blankHeatmap = np.zeros((M, N))

        self.heatmapPlot = pg.PlotItem()
        self.heatmapPlot.setAspectLocked(True)
        self.graphicsViewHeatmap.setCentralItem(self.heatmapPlot)     # GraphicsView is the main graphics container
        self.heatmapScatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 180))
        self.heatmapImage = pg.ImageItem(self.blankHeatmap)
        scaleFactor = self.allParameters.child('Pupil Camera').child('Scale Factor').value()
        self.heatmapImage.setLookupTable(self.colormap.getLookupTable())
        self.heatmapImage.translate(-0.5*M*scaleFactor, -0.5*N*scaleFactor)
        self.heatmapImage.scale(scaleFactor, scaleFactor)
        self.heatmapPlot.addItem(self.heatmapImage)
        self.heatmapPlot.autoRange(padding=0)
        self.heatmapScatterLastClicked = []
        self.heatmapScatter.sigClicked.connect(self.heatmapScatterplotClicked)
        self.heatmapPlot.addItem(self.heatmapScatter)
        for k in range(1, 4):
            (circX, circY) = self.drawCircle(k)
            self.circle = pg.PlotDataItem(circX,circY)
            self.circle.setPen(width=1, color='w')
            self.heatmapPlot.addItem(self.circle)
        (circX, circY) = self.drawCircle(4)
        self.circle = pg.PlotDataItem(circX,circY)
        self.circle.setPen(width=2, color='w')
        self.heatmapPlot.addItem(self.circle)
        self.heatmapVLine = pg.InfiniteLine(angle=90, movable=False)
        self.heatmapVLine.setPen(width=1, color='w')
        self.heatmapHLine = pg.InfiniteLine(angle=0, movable=False)
        self.heatmapHLine.setPen(width=1, color='w')
        self.heatmapPlot.addItem(self.heatmapVLine, ignoreBounds=True)
        self.heatmapPlot.addItem(self.heatmapHLine, ignoreBounds=True)
        self.heatmapVLine.setPos(0)
        self.heatmapHLine.setPos(0)

        # data view heatmap
        self.heatmapPlot_recording = pg.PlotItem()
        self.heatmapPlot_recording.setAspectLocked(True)
        self.graphicsViewHeatmapViewer.setCentralItem(self.heatmapPlot_recording)     # GraphicsView is the main graphics container
        self.heatmapScatter_recording = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 0, 0, 180))
        self.heatmapImage_recording = pg.ImageItem(self.blankHeatmap)
        self.heatmapImage_recording.setLookupTable(self.colormap.getLookupTable())
        self.heatmapImage_recording.translate(-0.5*M*scaleFactor, -0.5*N*scaleFactor)
        self.heatmapImage_recording.scale(scaleFactor, scaleFactor)
        self.heatmapPlot_recording.addItem(self.heatmapImage_recording)
        self.heatmapPlot_recording.autoRange(padding=0)    
        self.heatmapScatterLastClicked_recording = []
        self.heatmapScatter_recording.sigClicked.connect(self.heatmapScatterplotClicked)
        self.heatmapPlot_recording.addItem(self.heatmapScatter_recording)
        for k in range(1, 4):
            (circX, circY) = self.drawCircle(k)
            self.circle = pg.PlotDataItem(circX,circY)
            self.circle.setPen(width=1, color='w')
            self.heatmapPlot_recording.addItem(self.circle)
        (circX, circY) = self.drawCircle(4)
        self.circle = pg.PlotDataItem(circX,circY)
        self.circle.setPen(width=2, color='w')
        self.heatmapPlot_recording.addItem(self.circle)
        self.heatmapVLine_recording = pg.InfiniteLine(angle=90, movable=False)
        self.heatmapVLine_recording.setPen(width=1, color='w')
        self.heatmapHLine_recording = pg.InfiniteLine(angle=0, movable=False)
        self.heatmapHLine_recording.setPen(width=1, color='w')
        self.heatmapPlot_recording.addItem(self.heatmapVLine_recording, ignoreBounds=True)
        self.heatmapPlot_recording.addItem(self.heatmapHLine_recording, ignoreBounds=True)
        self.heatmapVLine_recording.setPos(0)
        self.heatmapHLine_recording.setPos(0)

        # Andor Live image - Sample
        self.SampleView = CustomViewBox(invertY=True)     
        self.graphicsViewSample.setCentralItem(self.SampleView)    
        self.SampleView.setAspectLocked(True)
        self.SampleImage = pg.ImageItem(np.zeros((1024, 170)))                        
        self.SampleView.addItem(self.SampleImage)
        self.SampleView.autoRange(padding=0)

        # Andor Live image - Reference
        self.CalibView = CustomViewBox(invertY=True)     
        self.graphicsViewCalib.setCentralItem(self.CalibView)    
        self.CalibView.setAspectLocked(True)
        self.CalibImage = pg.ImageItem(np.zeros((1024, 170)))                        
        self.CalibView.addItem(self.CalibImage)
        self.CalibView.autoRange(padding=0)

        # Andor recorded image
        self.EMCCDView_recording = CustomViewBox(invertY=True)                      
        self.dataViewEMCCD.setCentralItem(self.EMCCDView_recording)    
        self.EMCCDView_recording.setAspectLocked(True)
        self.EMCCDImage_recording = pg.ImageItem(np.zeros((1024, 170)))                         
        self.EMCCDView_recording.addItem(self.EMCCDImage_recording)
        self.EMCCDView_recording.autoRange(padding=0)

        self.maxScanPoints = 400  # Number of data points in view in freerunning mode

        # sampleSpectrumPlot is the plot of the raw data and Lorentzian fit
        self.sampleSpectrumPlot = pg.PlotItem()
        # self.sampleSpectrumPlot.setYRange(0, 17500)
        self.sampleSpectrumPlot.enableAutoRange(axis=self.sampleSpectrumPlot.vb.YAxis, enable=True)
        self.graphicsViewSampleSpectrum.setCentralItem(self.sampleSpectrumPlot)
        self.sampleSpectrumItem = pg.PlotDataItem(symbol='o')
        self.sampleSpectrumItem.setSymbolPen(color='g')
        self.sampleSpectrumItem.setPen(None)
        self.sampleSpectrumItem.setData(100*np.ones(100))
        self.sampleSpectrumItem2 = pg.PlotDataItem()
        self.sampleSpectrumItem2.setPen(width=2.5, color='r')
        self.sampleSpectrumItem2.setData(200*np.ones(100))
        self.sampleSpectrumPlot.addItem(self.sampleSpectrumItem)
        self.sampleSpectrumPlot.addItem(self.sampleSpectrumItem2)
        axBottom = self.sampleSpectrumPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleSpectrumPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        # calibSpectrumPlot is the plot of the raw data and Lorentzian fit
        self.calibSpectrumPlot = pg.PlotItem()
        # self.calibSpectrumPlot.setYRange(0, 17500)
        self.calibSpectrumPlot.enableAutoRange(axis=self.calibSpectrumPlot.vb.YAxis, enable=True)
        self.graphicsViewCalibSpectrum.setCentralItem(self.calibSpectrumPlot)
        self.calibSpectrumItem = pg.PlotDataItem(symbol='o')
        self.calibSpectrumItem.setSymbolPen(color='g')
        self.calibSpectrumItem.setPen(None)
        self.calibSpectrumItem.setData(100*np.ones(100))
        self.calibSpectrumItem2 = pg.PlotDataItem()
        self.calibSpectrumItem2.setPen(width=2.5, color='r')
        self.calibSpectrumItem2.setData(200*np.ones(100))
        self.calibSpectrumPlot.addItem(self.calibSpectrumItem)
        self.calibSpectrumPlot.addItem(self.calibSpectrumItem2)
        axBottom = self.calibSpectrumPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.calibSpectrumPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        # Data viewer tab
        self.singleSpectrumPlot_recording = pg.PlotItem()
        # self.singleSpectrumPlot_recording.setYRange(0, 17500)
        self.singleSpectrumPlot_recording.enableAutoRange(axis=self.singleSpectrumPlot_recording.vb.YAxis, enable=True)
        self.dataViewSingleSpectrum.setCentralItem(self.singleSpectrumPlot_recording)
        self.singleSpectrumItem_recording = pg.PlotDataItem(symbol='o')
        self.singleSpectrumItem_recording.setSymbolPen(color='g')
        self.singleSpectrumItem_recording.setData(100*np.ones(100))
        self.singleSpectrumItem2_recording = pg.PlotDataItem()
        self.singleSpectrumItem2_recording.setPen(width=2.5, color='r')
        self.singleSpectrumItem2_recording.setData(200*np.ones(100))
        self.singleSpectrumPlot_recording.addItem(self.singleSpectrumItem_recording)
        self.singleSpectrumPlot_recording.addItem(self.singleSpectrumItem2_recording)
        axBottom = self.singleSpectrumPlot_recording.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.singleSpectrumPlot_recording.getAxis('left')
        axLeft.setPen(width=2, color='w')

        # sampleScanDepthPlot is the Brillouin vs z axis plot
        self.sampleScanDepthPlot = pg.PlotItem()
        self.sampleScanDepthPlot.setYRange(5,6)
        self.graphicsViewSampleScanDepth.setCentralItem(self.sampleScanDepthPlot)
        self.sampleScanDepthPlot.enableAutoRange(axis=self.sampleScanDepthPlot.vb.XAxis, enable=True)
        self.sampleScanDepthItem = pg.PlotDataItem() 
        self.sampleScanDepthItem.setPen(width=2.5, color='g')
        self.sampleScanDepthItem2 = pg.PlotDataItem()
        self.sampleScanDepthItem2.setPen(width=2.5, color='b')
        self.sampleScanDepthPlot.addItem(self.sampleScanDepthItem)
        self.sampleScanDepthPlot.addItem(self.sampleScanDepthItem2)
        axBottom = self.sampleScanDepthPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleScanDepthPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')
        self.sampleScanDepthData = np.array([])
        self.sampleScanDepthData2 = np.array([])

        # sampleScanDepthPlot is the Brillouin vs z axis plot
        self.calibScanDepthPlot = pg.PlotItem()
        self.calibScanDepthPlot.setYRange(5,6)
        self.graphicsViewCalibScanDepth.setCentralItem(self.calibScanDepthPlot)
        self.calibScanDepthPlot.enableAutoRange(axis=self.calibScanDepthPlot.vb.XAxis, enable=True)
        self.calibScanDepthItem = pg.PlotDataItem() 
        self.calibScanDepthItem.setPen(width=2.5, color='g')
        self.calibScanDepthItem2 = pg.PlotDataItem()
        self.calibScanDepthItem2.setPen(width=2.5, color='b')
        self.calibScanDepthPlot.addItem(self.calibScanDepthItem)
        self.calibScanDepthPlot.addItem(self.calibScanDepthItem2)
        axBottom = self.calibScanDepthPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.calibScanDepthPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')
        self.calibScanDepthData = np.array([])
        self.calibScanDepthData2 = np.array([])

        # data viewer tab
        self.scanDepthPlot_recording = pg.PlotItem()
        self.scanDepthPlot_recording.setYRange(5,6)
        self.dataViewScanDepth.setCentralItem(self.scanDepthPlot_recording)
        self.scanDepthItem_recording = pg.PlotDataItem(symbol='o')
        self.scanDepthItem_recording.setPen(width=2.5, color='b')
        self.scanDepthPlot_recording.addItem(self.scanDepthItem_recording)
        self.scanDepthItem2_recording = pg.PlotDataItem()
        self.scanDepthItem2_recording.setPen(width=2.5, color='r')
        self.scanDepthPlot_recording.addItem(self.scanDepthItem2_recording)
        axBottom = self.scanDepthPlot_recording.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.scanDepthPlot_recording.getAxis('left')
        axLeft.setPen(width=2, color='w')

        # This is in the viewer tab
        self.specSeriesData_recording = np.zeros((self.maxScanPoints, 512))
        self.specSeriesPlot_recording = pg.PlotItem()
        self.dataViewSpecSeries.setCentralItem(self.specSeriesPlot_recording)
        self.specSeriesImage_recording = pg.ImageItem(self.specSeriesData_recording)
        self.specSeriesPlot_recording.addItem(self.specSeriesImage_recording)
        axBottom = self.specSeriesPlot_recording.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.specSeriesPlot_recording.getAxis('left')
        axLeft.setPen(width=2, color='w')

        # This is the sample spectrograph:
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 2048))
        self.sampleSpecSeriesSize = 0
        self.sampleSpecSeriesPlot = pg.PlotItem()
        self.graphicsViewSampleSpecSeries.setCentralItem(self.sampleSpecSeriesPlot)
        self.sampleSpecSeriesImage = pg.ImageItem(self.sampleSpecSeriesData)
        self.sampleSpecSeriesPlot.addItem(self.sampleSpecSeriesImage)
        axBottom = self.sampleSpecSeriesPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleSpecSeriesPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')
        # This is the reference spectrograph:
        self.calibSpecSeriesData = np.zeros((self.maxScanPoints, 2048))
        self.calibSpecSeriesSize = 0
        self.calibSpecSeriesPlot = pg.PlotItem()
        self.graphicsViewCalibSpecSeries.setCentralItem(self.calibSpecSeriesPlot)
        self.calibSpecSeriesImage = pg.ImageItem(self.calibSpecSeriesData)
        self.calibSpecSeriesPlot.addItem(self.calibSpecSeriesImage)
        axBottom = self.calibSpecSeriesPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.calibSpecSeriesPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        self.mainUI()

        self.MakoProcessThread.pupilRadius = self.allParameters.child('Pupil Camera').child('Pupil Radius').value()
        self.AndorProcessThread.channel = False # False = Sample, True = Reference
        self.AndorProcessThread.sampleSpectCenter = self.allParameters.child('Spectrometer Camera').child('Sample Column').value()
        self.AndorProcessThread.calibSpectCenter = self.allParameters.child('Spectrometer Camera').child('Ref. Column').value()
        self.AndorProcessThread.sampleSlineIdx = self.allParameters.child('Spectrometer Camera').child('Sample Row').value()
        self.AndorProcessThread.calibSlineIdx = self.allParameters.child('Spectrometer Camera').child('Ref. Row').value()
        self.AndorDeviceThread.start()
        self.AndorDeviceThread.setPriority(QtCore.QThread.TimeCriticalPriority)
        self.AndorProcessThread.start()
        time.sleep(0.1)   # CRUCIAL!! DON'T REMOVE. Need to let the threads fully start before continuing

        # create the figures for plotting Brillouin shifts and fits
        self.AndorProcessThread.updateSampleBrillouinSeqSig.connect(self.UpdateSampleBrillouinSeqPlot)
        self.AndorProcessThread.updateCalibBrillouinSeqSig.connect(self.UpdateCalibBrillouinSeqPlot)
        self.AndorProcessThread.updateSampleSpectrum.connect(self.UpdateSampleSpectrum)
        self.AndorProcessThread.updateCalibSpectrum.connect(self.UpdateCalibSpectrum)

        self.BrillouinScan = ScanManager(self.cancel_event, self.stop_event, self.ZaberDevice, self.ShutterDevice)
        self.BrillouinScan.addToSequentialList(self.AndorDeviceThread, self.AndorProcessThread)
        self.BrillouinScan.addToSequentialList(self.MakoDeviceThread, self.MakoProcessThread)
        self.BrillouinScan.addToSequentialList(self.TempSensorDeviceThread, self.TempSensorProcessThread)
        self.BrillouinScan.addToSequentialList(self.SynthDeviceThread, self.SynthProcessThread)
        self.BrillouinScan.finished.connect(self.onFinishScan)
        self.BrillouinScan.clearGUISig.connect(self.clearGUIElements)

        self.InitHardwareParameterTree()

    def drawCircle(self, radius, center=(0.0,0.0)):
        angle = np.linspace(0,2*math.pi,100)
        circleX = radius*np.cos(angle) + center[0]
        circleY = radius*np.sin(angle) + center[1]
        return (circleX, circleY)

    def CMOShLineValueChange(self, param, value):
        # print("[CMOShLineValueChange]")
        self.CMOShLine.setPos(value)
        self.configParser.set('Scan', 'laser_position_Y', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def CMOSvLineValueChange(self, param, value):
        # print("[CMOSvLineValueChange]")
        self.CMOSvLine.setPos(value)
        self.configParser.set('Scan', 'laser_position_X', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def sampleSpectCenterValueChange(self, param, value):
        # print("[sampleSpectCenterValueChange]")
        self.AndorProcessThread.sampleSpectCenter = self.allParameters.child('Spectrometer Camera').child('Sample Column').value()
        self.configParser.set('Andor', 'sampleSpectCenter', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)
    def calibSpectCenterValueChange(self, param, value):
        # print("[calibSpectCenterValueChange]")
        self.AndorProcessThread.calibSpectCenter = self.allParameters.child('Spectrometer Camera').child('Ref. Column').value()
        self.configParser.set('Andor', 'calibSpectCenter', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def sampleSlineIdxValueChange(self, param, value):
        self.AndorProcessThread.sampleSlineIdx = self.allParameters.child('Spectrometer Camera').child('Sample Row').value()
        self.configParser.set('Andor', 'sampleSlineIdx', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)
    def calibSlineIdxValueChange(self, param, value):
        self.AndorProcessThread.calibSlineIdx = self.allParameters.child('Spectrometer Camera').child('Ref. Row').value()
        self.configParser.set('Andor', 'calibSlineIdx', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def pupilRadiusValueChange(self, param, value):
        # print("[pupilRadiusValueChange]")
        self.MakoProcessThread.pupilRadius = self.allParameters.child('Pupil Camera').child('Pupil Radius').value()
        self.configParser.set('Mako', 'pupilRadius', str(float(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    #############################################################################################
    # This next group of methods are used to set/get hardware settings, using the parameterTree #
    #############################################################################################
    def InitHardwareParameterTree(self):
        # print("[InitHardwareParameterTree]")

        # ========================= Andor Camera ================================
        pItem = self.allParameters.child('Spectrometer Camera')
        pItem.child('AutoExposure').sigActivated.connect(self.switchAutoExp)
        pItem.child('Camera Temp.').setValue(self.AndorDeviceThread.getTemperature())
        pItem.child('Exposure').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setExposure))
        pItem.child('Exposure').setValue(self.AndorDeviceThread.getExposure())

        # ========================= Microwave Source ================================
        pItem = self.allParameters.child('Microwave Source')
        pItem.child('RF Frequency').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.SynthDeviceThread.setFreq))
        pItem.child('RF Frequency').setValue(self.SynthDeviceThread.getFreq())
        pItem.child('RF Power').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.SynthDeviceThread.setPower))
        pItem.child('RF Power').setValue(self.SynthDeviceThread.getPower())

        # ========================= Motor =================================
        pItem = self.allParameters.child('Motor')
        self.MotorPositionUpdate()
        
        # connect Jog Buttons
        motorFwdFunX = lambda: self.ZaberDevice.moveRelative( 
            'x', self.allParameters.child('Motor').child('Jog step').value())
        motorBackFunX = lambda: self.ZaberDevice.moveRelative( 
            'x', -self.allParameters.child('Motor').child('Jog step').value())
        pItem.child('Jog X').sigActivated.connect(motorFwdFunX)
        pItem.child('Jog X').sigActivated2.connect(motorBackFunX)
        pItem.child('Jog X').sigActivated3.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome', 'x'))

        motorFwdFunY = lambda: self.ZaberDevice.moveRelative( 
            'y', self.allParameters.child('Motor').child('Jog step').value())
        motorBackFunY = lambda: self.ZaberDevice.moveRelative( 
            'y', -self.allParameters.child('Motor').child('Jog step').value())
        pItem.child('Jog Y').sigActivated.connect(motorFwdFunY)
        pItem.child('Jog Y').sigActivated2.connect(motorBackFunY)
        pItem.child('Jog Y').sigActivated3.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome', 'y'))

        motorFwdFunZ = lambda: self.ZaberDevice.moveRelative( 
            'z', self.allParameters.child('Motor').child('Jog step').value())
        motorBackFunZ = lambda: self.ZaberDevice.moveRelative( 
            'z', -self.allParameters.child('Motor').child('Jog step').value())
        pItem.child('Jog Z').sigActivated.connect(motorFwdFunZ)
        pItem.child('Jog Z').sigActivated2.connect(motorBackFunZ)
        pItem.child('Jog Z').sigActivated3.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome', 'z'))

        # connect Move and Home buttons
        motorMoveFunX = lambda: self.ZaberDevice.moveAbs(
            'x', self.allParameters.child('Motor').child('Move to location').value())
        pItem.child('Move').sigActivated.connect(motorMoveFunX)
        motorMoveFunY = lambda: self.ZaberDevice.moveAbs(
            'y', self.allParameters.child('Motor').child('Move to location').value())
        pItem.child('Move').sigActivated2.connect(motorMoveFunY)
        motorMoveFunZ = lambda: self.ZaberDevice.moveAbs(
            'z', self.allParameters.child('Motor').child('Move to location').value())
        pItem.child('Move').sigActivated3.connect(motorMoveFunZ)

        # ========================= Scan ===================
        pItem = self.allParameters.child('Scan')
        startPosX = pItem.child('Start Position').child('X').value()
        stepSizeX = pItem.child('Step Size').child('X').value()
        frameNumX = pItem.child('Frame Number').child('X').value()
        endPosX = startPosX + stepSizeX * frameNumX
        pItem.child('End Position').child('X').setValue(endPosX)
        pItem.child('Start Position').child('X').sigValueChanging.connect(lambda param, value: self.updateScanEndPosX(0, param, value))
        pItem.child('Step Size').child('X').sigValueChanging.connect(lambda param, value: self.updateScanEndPosX(1, param, value))
        pItem.child('Frame Number').child('X').sigValueChanging.connect(lambda param, value: self.updateScanEndPosX(2, param, value))
        startPosY = pItem.child('Start Position').child('Y').value()
        stepSizeY = pItem.child('Step Size').child('Y').value()
        frameNumY = pItem.child('Frame Number').child('Y').value()
        endPosY = startPosY + stepSizeY * frameNumY
        pItem.child('End Position').child('Y').setValue(endPosY)
        pItem.child('Start Position').child('Y').sigValueChanging.connect(lambda param, value: self.updateScanEndPosY(0, param, value))
        pItem.child('Step Size').child('Y').sigValueChanging.connect(lambda param, value: self.updateScanEndPosY(1, param, value))
        pItem.child('Frame Number').child('Y').sigValueChanging.connect(lambda param, value: self.updateScanEndPosY(2, param, value))
        startPosZ = pItem.child('Start Position').child('Z').value()
        stepSizeZ = pItem.child('Step Size').child('Z').value()
        frameNumZ = pItem.child('Frame Number').child('Z').value()
        endPosZ = startPosZ + stepSizeZ * frameNumZ
        pItem.child('End Position').child('Z').setValue(endPosZ)
        pItem.child('Start Position').child('Z').sigValueChanging.connect(lambda param, value: self.updateScanEndPosZ(0, param, value))
        pItem.child('Step Size').child('Z').sigValueChanging.connect(lambda param, value: self.updateScanEndPosZ(1, param, value))
        pItem.child('Frame Number').child('Z').sigValueChanging.connect(lambda param, value: self.updateScanEndPosZ(2, param, value))
        pItem.child('Scan/Cancel').sigActivated.connect(self.startScan)
        pItem.child('Scan/Cancel').sigActivated2.connect(self.cancelScan)

        # ========================= Sample/Reference ===================
        pItem = self.allParameters.child('Scan')
        pItem.child('ToggleReference').sigActivated.connect(self.toggleReference)
        
        # ========================= Hardware monitor timers ===================

        self.hardwareGetTimer = QTimer()    # TODO: disable timer when scan starting
        self.hardwareGetTimer.timeout.connect(self.HardwareParamUpdate)
        self.hardwareGetTimer.start(10000)

        # Separate timer for motor position, update at faster rate
        self.motorPositionTimer = QTimer()
        self.motorPositionTimer.timeout.connect(self.MotorPositionUpdate)
        self.motorPositionTimer.start(500)

    def changeHardwareSetting(self, data, funcHandle):
        # print("[changeHardwareSetting]")
        funcHandle(data.value())

    def HardwareParamUpdate(self):
        # print("[HardwareParamUpdate]")
        temp = self.AndorDeviceThread.getTemperature()
        self.allParameters.child('Spectrometer Camera').child('Camera Temp.').setValue(temp)
        # if (self.ShutterDevice.state == ShutterDevice.SAMPLE_STATE):
            # expTime = self.AndorDeviceThread.getExposure()
            # self.allParameters.child('Spectrometer Camera').child('Exposure').setValue(expTime)

    def updateScanEndPosX(self, updateIndex, param, value):
        print("[updateScanEndPosX]")
        pItem = self.allParameters.child('Scan')
        startPos = pItem.child('Start Position').child('X').value()
        stepSize = pItem.child('Step Size').child('X').value()
        frameNum = pItem.child('Frame Number').child('X').value()
        if (updateIndex == 0):
            startPos = value
        elif (updateIndex == 1):
            stepSize = value
        else:
            frameNum = value
        endPos = startPos + stepSize * (frameNum - 1)
        pItem.child('End Position').child('X').setValue(endPos)

    def updateScanEndPosY(self, updateIndex, param, value):
        print("[updateScanEndPosY]")
        pItem = self.allParameters.child('Scan')
        startPos = pItem.child('Start Position').child('Y').value()
        stepSize = pItem.child('Step Size').child('Y').value()
        frameNum = pItem.child('Frame Number').child('Y').value()
        if (updateIndex == 0):
            startPos = value
        elif (updateIndex == 1):
            stepSize = value
        else:
            frameNum = value
        endPos = startPos + stepSize * (frameNum - 1)
        pItem.child('End Position').child('Y').setValue(endPos)

    def updateScanEndPosZ(self, updateIndex, param, value):
        print("[updateScanEndPosZ]")
        pItem = self.allParameters.child('Scan')
        startPos = pItem.child('Start Position').child('Z').value()
        stepSize = pItem.child('Step Size').child('Z').value()
        frameNum = pItem.child('Frame Number').child('Z').value()
        if (updateIndex == 0):
            startPos = value
        elif (updateIndex == 1):
            stepSize = value
        else:
            frameNum = value
        endPos = startPos + stepSize * (frameNum - 1)
        pItem.child('End Position').child('Z').setValue(endPos)

    @QtCore.pyqtSlot(list)
    def MotorPositionUpdate2(self, pos):
        #print("[MotorPositionUpdate2]")
        self.allParameters.child('Motor').child('Current X location').setValue(pos[0])
        self.allParameters.child('Motor').child('Current Y location').setValue(pos[1])
        self.allParameters.child('Motor').child('Current Z location').setValue(pos[2])

    def MotorPositionUpdate(self):
        #print("[MotorPositionUpdate]")
        pos = self.ZaberDevice.updatePosition()
        self.allParameters.child('Motor').child('Current X location').setValue(pos[0])
        self.allParameters.child('Motor').child('Current Y location').setValue(pos[1])
        self.allParameters.child('Motor').child('Current Z location').setValue(pos[2])

    @QtCore.pyqtSlot()
    def clearGUIElements(self):
        # print("[clearGUIElements]")
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 2048))
        self.calibSpecSeriesData = np.zeros((self.maxScanPoints, 2048))
        self.sampleSpecSeriesSize = 0
        self.calibSpecSeriesSize = 0
        self.sampleScanDepthData = np.array([])
        self.sampleScanDepthData2 = np.array([])
        self.calibScanDepthData = np.array([])
        self.calibScanDepthData2 = np.array([])

    def startScan(self):
        # First check that a session is running, and that an experiment is selected
        if self.session is None:
            choice = QtGui.QMessageBox.warning(self, 'Starting Scan...',
                                                "No Session open!",
                                                QtGui.QMessageBox.Ok)
            return

        print("Starting a scan in Exp_%d: " % self.model.activeExperiment)

        # take screenshot
        p = QtGui.QPixmap.grabWindow(self.winId())
        pImage = p.toImage()
        channels = 4
        s = pImage.bits().asstring(p.width() * p.height() * channels)
        screenshotArr = np.fromstring(s, dtype=np.uint8).reshape((p.height(), p.width(), channels))


        flattenedParamList = generateParameterList(self.params, self.allParameters)

        scanSettings = {'start': self.allParameters.child('Scan').child('Start Position').value(),  
            'step': self.allParameters.child('Scan').child('Step Size').value(),   
            'frames': self.allParameters.child('Scan').child('Frame Number').value(),
            'laserX': self.allParameters.child('Scan').child('More Settings').child('Laser Focus X').value(),
            'laserY': self.allParameters.child('Scan').child('More Settings').child('Laser Focus Y').value(),
            'scaleFactor': self.allParameters.child('Pupil Camera').child('Scale Factor').value(),
            'refExp': self.allParameters.child('Spectrometer Camera').child('Ref. Exposure').value(),
            'sampleExp': self.allParameters.child('Spectrometer Camera').child('Exposure').value(),
            'waterConst': self.waterConst, 'plasticConst': self.plasticConst,
            'screenshot': screenshotArr,
            'flattenedParamList': flattenedParamList }
        self.BrillouinScan.assignScanSettings(scanSettings)

        self.maxScanPoints = scanSettings['frames']+5 # Scale plot window to scan length (+ 5 calibration frames)

        self.BrillouinScan.saveScan = True
        self.BrillouinScan.sessionData = self.session
        self.BrillouinScan.saveExpIndex = self.model.activeExperiment

        # Stop periodic polling of hardware state
        self.hardwareGetTimer.stop()
        self.motorPositionTimer.stop()
        self.BrillouinScan.motorPosUpdateSig.connect(self.MotorPositionUpdate2)

        # Finally start scan in a new thread
        self.BrillouinScan.start()

    def cancelScan(self):
        print('Stopping current scan and wrapping-up.')
        self.cancel_event.set()

    def onFinishScan(self):
        self.maxScanPoints = 400 # Re-scale plot window for free-running mode
        if (self.allParameters.child('Scan').child('ToggleReference').value() == True):
            self.ShutterDevice.setShutterState(self.ShutterDevice.REFERENCE_STATE)
        else:
            self.ShutterDevice.setShutterState(self.ShutterDevice.SAMPLE_STATE)
        currExp = self.session.experimentList[self.BrillouinScan.saveExpIndex]
        laserCoords = np.array([np.float(self.allParameters.child('Scan').child('More Settings').child('Laser Focus X').value()), \
            np.float(self.allParameters.child('Scan').child('More Settings').child('Laser Focus Y').value())])
        coords = currExp.getMeanScanCoords(laserCoords)
        BS = currExp.getBS()
        scanIndices = currExp.getActiveScanIndices()
        self.updateHeatmap(coords, BS, scanIndices, self.BrillouinScan.saveExpIndex)
        self.makoPoints = coords

        #TODO: highlight corresponding item on self.treeView

        self.BrillouinScan.motorPosUpdateSig.disconnect(self.MotorPositionUpdate2)
        self.hardwareGetTimer.start(10000)
        self.motorPositionTimer.start(500)
        self.cancel_event.clear()
        print('Scan completed')

    def toggleReference(self, sliderParam, state):
        # print("[toggleReference]")
        # state == True --> Reference
        # state == False --> Sample
        if state:
            self.ShutterDevice.setShutterState(self.ShutterDevice.REFERENCE_STATE)
            self.AndorProcessThread.channel = True
            self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Ref. Exposure').value())
        else:
            self.ShutterDevice.setShutterState(self.ShutterDevice.SAMPLE_STATE)
            self.AndorProcessThread.channel = False
            self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())

    def switchAutoExp(self, sliderParam, state):
        if state:
            self.AndorDeviceThread.setAutoExp(True)
            print("Spectrometer auto exposure ON")
        else:
            self.AndorDeviceThread.setAutoExp(False)
            self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())
            print("Spectrometer auto exposure OFF")
            # self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())

    #############################################################################################
    # This next group of methods callback methods to display acquired data #
    #############################################################################################

    def updateHeatmapByExp(self, expIdx):
        # print("[updateHeatmapByExp]")
        currExp = self.session.experimentList[expIdx]
        laserCoords = np.array([np.float(self.allParameters.child('Scan').child('More Settings').child('Laser Focus X').value()), \
            np.float(self.allParameters.child('Scan').child('More Settings').child('Laser Focus Y').value())])
        coords = currExp.getMeanScanCoords(laserCoords)
        self.makoPoints = coords
        BS = currExp.getBS()
        activeScanIdx = currExp.getActiveScanIndices()
        self.updateHeatmap(coords, BS, activeScanIdx, expIdx)

    def updateHeatmapBySelection(self, idx):
        # print("[updateHeatmapBySelection]")
        if self.dataViewerTab.currentIndex() == 0:  #acq tab
            lastClicked = self.heatmapScatterLastClicked
            heatmapScatter = self.heatmapScatter
        else:   #viewer tab
            lastClicked = self.heatmapScatterLastClicked_recording
            heatmapScatter = self.heatmapScatter_recording

        item = self.model.itemFromIndex(idx)
        
        if item.parent() is None:       # Experiment selected (not scan)
            return
        expIdx = item.parent().row()
        scanIdx = item.row()

        for p in heatmapScatter.points():
            print(p.data())

        # if desired selection is already highlighted, then do nothing
        if len(lastClicked)==1:     
            p = lastClicked[0]
            if p.data()[0] == expIdx and p.data()[1] == scanIdx:
                return

        # clear all previous selections
        for p in lastClicked:
            p.resetPen()
        lastClicked[:] = []

        #for p in heatmapScatter.points():

        # iterate through all scatter points until the desired one is found 
        # print("expIdx = %d, scanIdx = %d" % (expIdx, scanIdx))
        p = None
        for p1 in heatmapScatter.points():
            if p1.data()[0] == expIdx and p1.data()[1] == scanIdx:
                p = p1
                continue
        if p is None:   # point not found if scan is deleted
            return

        # highlight the found point
        # print("pen found")
        p.setPen('w', width=2)
        lastClicked[:] = [p]    # only allow single selection   


    def updateHeatmap(self, scanPoints, BS, activeScanIndices, expIndex):
        # print("[updateHeatmap]")
        if self.dataViewerTab.currentIndex() == 0:  #acq tab
            lastClicked = self.heatmapScatterLastClicked
            heatmapImage = self.heatmapImage
        else:   #viewer tab
            lastClicked = self.heatmapScatterLastClicked_recording
            heatmapImage = self.heatmapImage_recording

        scaleFactor = self.allParameters.child('Pupil Camera').child('Scale Factor').value()
        scanPointsMM = scaleFactor*scanPoints
        BSArr = (np.transpose(np.array([BS])))[:,0]
        cleanDatInd = (np.argwhere(~np.isnan(BSArr)))[:,0] # Remove NaNs
        scanPointsClean = (scanPoints[cleanDatInd])[0:]
        BSClean = BSArr[cleanDatInd]
        if len(BSClean) > 2:
            scanPointsX = scanPointsClean[:,0]
            scanPointsY = scanPointsClean[:,1]
            scanPointsZ = (np.transpose(BSClean))
            try:
                gridImage = griddata((scanPointsX, scanPointsY), scanPointsZ, (self.gridx, self.gridy), \
                    fill_value=0, method='cubic')
            except:
                print('[updateHeatmap] Could not generate heatmap')
                gridImage = self.blankHeatmap
        else:
            gridImage = self.blankHeatmap
        heatmapImage.setImage(gridImage, levels=(self.colormapLow, self.colormapHigh))
        # spots = [{'pos': scanPoints[i], 'data': (expIndex, i, BS[i])} for i in range(len(scanPoints))]
        badDatInd = (np.argwhere(np.isnan(scanPointsMM)))[:,0] # Find the NaNs
        scanPointsMM[badDatInd] = 0.0 # Replace NaNs with 0 for plotting
        spots = [{'pos': pos, 'data': (expIndex, i, b)} for (pos, b, i) in zip(scanPointsMM, BS, activeScanIndices)]

        lastClicked[:] = []

        self.heatmapScatter.setData(spots)
        self.heatmapScatter_recording.setData(spots)

        allSelected = self.treeView.selectionModel().selectedIndexes()
        if len(allSelected)>0:
            lastSelected = allSelected[0]
            self.updateHeatmapBySelection(lastSelected)
        
    def heatmapScatterplotClicked(self, plot, points):
        # print("[heatmapScatterplotClicked]")
        if self.dataViewerTab.currentIndex() == 0:  #acq tab
            lastClicked = self.heatmapScatterLastClicked
        else:   #viewer tab
            lastClicked = self.heatmapScatterLastClicked_recording

        for p in lastClicked:
            p.resetPen()
        if (points!=[]):
            p = points[0]
            print("clicked points", p)
            p.setPen('w', width=2)

            lastClicked[:] = [p]    # only allow single selection   

            # highlight the corresponding item in treeView
            expItem = self.model.item(p.data()[0])
            self.treeView.expand(expItem.index())
            newScanTreeIndex = expItem.child(p.data()[1],0).index()
            self.treeView.selectionModel().select(
                newScanTreeIndex, QtGui.QItemSelectionModel.ClearAndSelect|QtGui.QItemSelectionModel.Rows)
        else:
            lastClicked[:] = []

    def UpdateAmbientTemp(self, temperature):
        # print('UpdateAmbientTemp')
        self.allParameters.child('Scan').child('Ambient Temp.').setValue(temperature)

    # updates the figure containing the Brillouin sequence. newData is a list
    def UpdateSampleBrillouinSeqPlot(self, interPeakDist):
        # print("[UpdateBrillouinSeqPlot]")
        SD = self.allParameters.child('Scan').child('Ref. SD').value()
        FSR = self.allParameters.child('Scan').child('Ref. FSR').value()
        T = self.allParameters.child('Scan').child('Ambient Temp.').value()

        if len(interPeakDist)==2:
            newData = [0.5*(FSR - SD*interPeakDist[1])]
            newData2 = newData
        elif len(interPeakDist)==3:
            ## CALIBRATION if reference arm open and signal counts in optimal range
            if (self.ShutterDevice.state == ShutterDevice.REFERENCE_STATE and interPeakDist[0]<15900 and interPeakDist[0]>10000):
                print("Updating calibration...")
                WaterBS = self.waterConst[0]*T*T + self.waterConst[1]*T + self.waterConst[2]
                PlasticBS = 16.3291 - (self.plasticConst[0]*T*T + self.plasticConst[1]*T + self.plasticConst[2])
                SD = 2*(PlasticBS - WaterBS)/(interPeakDist[1] + interPeakDist[2])
                FSR = 2*WaterBS + interPeakDist[1]*SD
                self.allParameters.child('Scan').child('Ref. SD').setValue(SD)
                self.allParameters.child('Scan').child('Ref. FSR').setValue(FSR)
            newData = [0.5*(FSR - SD*interPeakDist[1])]
            newData2 = [0.5*(FSR - SD*interPeakDist[2])]
        else:
            newData = [np.nan]
            newData2 = [np.nan]

        if len(newData)+len(self.sampleScanDepthData) > self.maxScanPoints:
            t = self.maxScanPoints - len(newData) - len(self.sampleScanDepthData)
            self.sampleScanDepthData = np.roll(self.sampleScanDepthData, t)
            self.sampleScanDepthData[self.maxScanPoints - len(newData):] = newData
        else:
            self.sampleScanDepthData = np.append(self.sampleScanDepthData, newData)

        xdata = np.arange(len(self.sampleScanDepthData))
        xdata = xdata[~np.isnan(self.sampleScanDepthData)]
        ydata = self.sampleScanDepthData[~np.isnan(self.sampleScanDepthData)]
        self.sampleScanDepthItem.setData(x=xdata, y=ydata)

        if len(newData2)+len(self.sampleScanDepthData2) > self.maxScanPoints:
            t = self.maxScanPoints - len(newData2) - len(self.sampleScanDepthData2)
            self.sampleScanDepthData2 = np.roll(self.sampleScanDepthData2, t)
            self.sampleScanDepthData2[self.maxScanPoints - len(newData2):] = newData2
        else:
            self.sampleScanDepthData2 = np.append(self.sampleScanDepthData2, newData2)
        xdata2 = np.arange(len(self.sampleScanDepthData2))
        xdata2 = xdata2[~np.isnan(self.sampleScanDepthData2)]
        ydata2 = self.sampleScanDepthData2[~np.isnan(self.sampleScanDepthData2)]
        self.sampleScanDepthItem2.setData(x=xdata2, y=ydata2)

    # updates the figure containing the Brillouin value plot. newData is a list
    def UpdateCalibBrillouinSeqPlot(self, interPeakDist):
        # print("[UpdateBrillouinSeqPlot]")
        SD = self.allParameters.child('Scan').child('Ref. SD').value()
        FSR = self.allParameters.child('Scan').child('Ref. FSR').value()
        T = self.allParameters.child('Scan').child('Ambient Temp.').value()

        if len(interPeakDist)==2:
            newData = [0.5*(FSR - SD*interPeakDist[1])]
            newData2 = newData
        elif len(interPeakDist)==3:
            ## CALIBRATION if reference arm open and signal counts in optimal range
            if (self.ShutterDevice.state == ShutterDevice.REFERENCE_STATE and interPeakDist[0]<15900 and interPeakDist[0]>10000):
                print("Updating calibration...")
                WaterBS = self.waterConst[0]*T*T + self.waterConst[1]*T + self.waterConst[2]
                PlasticBS = 16.3291 - (self.plasticConst[0]*T*T + self.plasticConst[1]*T + self.plasticConst[2])
                SD = 2*(PlasticBS - WaterBS)/(interPeakDist[1] + interPeakDist[2])
                FSR = 2*WaterBS + interPeakDist[1]*SD
                self.allParameters.child('Scan').child('Ref. SD').setValue(SD)
                self.allParameters.child('Scan').child('Ref. FSR').setValue(FSR)
            newData = [0.5*(FSR - SD*interPeakDist[1])]
            newData2 = [0.5*(FSR - SD*interPeakDist[2])]
        else:
            newData = [np.nan]
            newData2 = [np.nan]

        if len(newData)+len(self.calibScanDepthData) > self.maxScanPoints:
            t = self.maxScanPoints - len(newData) - len(self.calibScanDepthData)
            self.calibScanDepthData = np.roll(self.calibScanDepthData, t)
            self.calibScanDepthData[self.maxScanPoints - len(newData):] = newData
        else:
            self.calibScanDepthData = np.append(self.calibScanDepthData, newData)

        xdata = np.arange(len(self.calibScanDepthData))
        xdata = xdata[~np.isnan(self.calibScanDepthData)]
        ydata = self.calibScanDepthData[~np.isnan(self.calibScanDepthData)]
        self.calibScanDepthItem.setData(x=xdata, y=ydata)

        if len(newData2)+len(self.calibScanDepthData2) > self.maxScanPoints:
            t = self.maxScanPoints - len(newData2) - len(self.calibScanDepthData2)
            self.calibScanDepthData2 = np.roll(self.calibScanDepthData2, t)
            self.calibScanDepthData2[self.maxScanPoints - len(newData2):] = newData2
        else:
            self.calibScanDepthData2 = np.append(self.calibScanDepthData2, newData2)
        xdata2 = np.arange(len(self.calibScanDepthData2))
        xdata2 = xdata2[~np.isnan(self.calibScanDepthData2)]
        ydata2 = self.calibScanDepthData2[~np.isnan(self.calibScanDepthData2)]
        self.calibScanDepthItem2.setData(x=xdata2, y=ydata2)

    # Plot fitted Brillouin spectrum
    # curvedata is a tuple of (raw spectrum, fitted spectrum)
    def UpdateSampleSpectrum(self,curveData):
        # print("[UpdateSpectrum]")
        rawSpect = curveData[0]
        fitSpect = curveData[1]

        xdata = np.arange(len(rawSpect))

        self.sampleSpectrumItem.setData(x=xdata, y=np.array(rawSpect))
        if len(fitSpect[~np.isnan(fitSpect)]) == len(rawSpect):
            self.sampleSpectrumItem2.setData(x=np.copy(xdata), y=np.array(fitSpect))
        else:
            self.sampleSpectrumItem2.setData(x=np.copy(xdata), y=np.zeros(len(rawSpect)))

        if self.sampleSpecSeriesData.shape[1]!=len(rawSpect):    #if size of spectrum change, reset the data
            # print('[UpdateSpectrum] Resizing spectrograph')
            self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, len(rawSpect)))
            self.sampleSpecSeriesSize = 0

        if self.sampleSpecSeriesSize < self.maxScanPoints:
            self.sampleSpecSeriesData[self.sampleSpecSeriesSize,:] = rawSpect
            self.sampleSpecSeriesSize += 1
        else:
            self.sampleSpecSeriesData = np.roll(self.sampleSpecSeriesData, -1, axis=0)
            self.sampleSpecSeriesData[-1, :] = rawSpect

        maximum = self.sampleSpecSeriesData.max()
        sampleSpecSeriesDataScaled = self.sampleSpecSeriesData #* (255.0 / maximum)
        self.sampleSpecSeriesImage.setImage(sampleSpecSeriesDataScaled)

    # Plot fitted Brillouin spectrum
    # curvedata is a tuple of (raw spectrum, fitted spectrum)
    def UpdateCalibSpectrum(self,curveData):
        # print("[UpdateSpectrum]")
        rawSpect = curveData[0]
        fitSpect = curveData[1]

        xdata = np.arange(len(rawSpect))

        self.calibSpectrumItem.setData(x=xdata, y=np.array(rawSpect))
        if len(fitSpect[~np.isnan(fitSpect)]) == len(rawSpect):
            self.calibSpectrumItem2.setData(x=np.copy(xdata), y=np.array(fitSpect))
        else:
            self.calibSpectrumItem2.setData(x=np.copy(xdata), y=np.zeros(len(rawSpect)))

        if self.calibSpecSeriesData.shape[1]!=len(rawSpect):    #if size of spectrum change, reset the data
            # print('[UpdateSpectrum] Resizing spectrograph')
            self.calibSpecSeriesData = np.zeros((self.maxScanPoints, len(rawSpect)))
            self.calibSpecSeriesSize = 0

        if (self.calibSpecSeriesSize < self.maxScanPoints):
            self.calibSpecSeriesData[self.calibSpecSeriesSize,:] = rawSpect
            self.calibSpecSeriesSize += 1
        else:
            self.calibSpecSeriesData = np.roll(self.calibSpecSeriesData, -1, axis=0)
            self.calibSpecSeriesData[-1, :] = rawSpect

        maximum = self.calibSpecSeriesData.max()
        calibSpecSeriesDataScaled = self.calibSpecSeriesData #* (255.0 / maximum)
        self.calibSpecSeriesImage.setImage(calibSpecSeriesDataScaled)


    # Update raw Andor image
    def AndorSampleProcessUpdate(self, image):
        # print("[AndorProcessUpdate]")
        #(counter, image) = self.AndorProcessThread.processedData.get()
        # img_rect = QtCore.QRectF(0, 0, 1024, 170)
        # self.SampleImage.setRect(img_rect)
        self.SampleImage.setImage(image.transpose((1,0)))

    # Update raw Andor image
    def AndorCalibProcessUpdate(self, image):
        # print("[AndorProcessUpdate]")
        #(counter, image) = self.AndorProcessThread.processedData.get()
        # img_rect = QtCore.QRectF(0, 0, 1024, 170)
        # self.SampleImage.setRect(img_rect)
        self.CalibImage.setImage(image.transpose((1,0)))

    # Update the CMOS camera image
    # makoData[0] is the CMOS camera image with pupil detection
    # makoData[1] is a tuple with the pupil center coordinate
    def MakoProcessUpdate(self, makoData):
        # print("[MakoProcessUpdate]")
        image = makoData[0]
        center = np.array([makoData[1]])
        # center[1] = self.MakoDeviceThread.imageHeight - center[1]
        # (counter, data) = self.MakoProcessThread.processedData.get()
        # print('self.makoPoints =', self.makoPoints)
        # print('center =', center)
        if len(self.makoPoints[~np.isnan(self.makoPoints)])>0 and np.all(~np.isnan(center)):
            adjPoints = self.makoPoints + center
            spots = [{'pos': pos} for pos in adjPoints]
            self.CMOSScatter.setData(spots)
        else:
            self.CMOSScatter.clear()
        self.CMOSImage.setImage(image)

    def createNewSession(self):
        dialog = QtGui.QFileDialog()
        options = QtGui.QFileDialog.Options()
        options |= QtGui.QFileDialog.DontUseNativeDialog
        dialog.setOptions(options)
        dialog.setDefaultSuffix('hdf5')
        dialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
        dialog.setNameFilter("HDF5 Files(*.hdf5)")
        if dialog.exec_():
            filename = str(dialog.selectedFiles().first())
        else:
            return

        # filename = dialog.getSaveFileName(self,"Create a new session file","","HDF5 Files(*.hdf5)", options=options)
        # print(filename)

        # if filename:
        #     filename = str(filename)
        #     fileBase, fileExtension = os.path.splitext(str(filename))
        #     if (fileExtension.lower() != 'hdf5'.lower()):
        #         filename = filename + '.hdf5'
        # else:
        #     return

        self.dataFileName = filename
        self.sessionName.setText(QtCore.str(filename))

        #create a single new session
        self.session = SessionData(ntpath.basename(self.dataFileName), filename=filename)
        self.model.session = self.session

        self.session.updateTreeViewSig.connect(
            lambda updateIndices:self.model.updateTree(updateIndices, self.treeView))
        self.session.addNewExperiment()

        self.treeView.setEnabled(True)


    def closeEvent(self,event):
        print("Program Shutdown")
        if self.dataFile:
            self.dataFile.close()

        self.hardwareGetTimer.stop()
        self.motorPositionTimer.stop()

        self.stop_event.set()
        while self.AndorDeviceThread.isRunning():
            time.sleep(0.1)
        while self.MakoDeviceThread.isRunning():
            time.sleep(0.1)

        self.MakoDeviceThread.shutdown()
        self.TempSensorDeviceThread.shutdown()
        self.ZaberDevice.shutdown()
        self.ShutterDevice.shutdown()
        self.SynthDeviceThread.shutdown()

        event.accept() #closes the application

    #############################################################################################
    # This next group of methods are for dataViewer widgets                                     #
    #############################################################################################
    def dataViewerTabChanged(self, index):
        if index == 0:
            # in data acq tab, disconnect data view tab widget signals
            if self.validDataView:
                self.dataViewSlider.valueChanged.disconnect()   

        print("[Data Viewer Tab] Open")
        

        # Find which dataset is selected
        allSelected = self.treeView.selectionModel().selectedIndexes()
        if len(allSelected)>0:
            lastSelected = allSelected[0]
            # self.updateHeatmapBySelection(lastSelected)
            item = self.model.itemFromIndex(lastSelected)

            if item.parent() is None:       # Experiment selected (not scan)
                expIdx = item.row()
                if (item.rowCount() == 0):  # No scan in experiment
                    print("[dataViewerTabChanged] no scan in experiment")
                    return
                scanIdx = 0
            else:
                expIdx = item.parent().row()
                scanIdx = item.row()        
                print("[dataViewerTabChanged] Selected exp_%d scan_%d" % (expIdx, scanIdx))
        else:
            return

        # heatmap from any experiment can be displayed in dataView tab, but only active
        # experiemnt heatmap is shown in dataAcq tab
        if (index == 1):
            self.updateHeatmapByExp(expIdx)
        else:
            self.updateHeatmapByExp(self.model.activeExperiment)

        self.validDataView = True

        self.currScan = self.session.experimentList[expIdx].scanList[scanIdx]

        # Set up the data slider
        self.dataViewSlider.setMinimum(0)
        self.dataViewSlider.setMaximum(self.currScan.EMCCDImage.shape[0]-1)
        self.dataViewSlider.setValue(0)
        self.dataViewInfoDisplay.setText('Frame {}/{}'.format(0, self.dataViewSlider.maximum()) )
        self.dataViewSlider.valueChanged.connect(self.onDataViewSliderChange)

        # Display the requested dataset
        self.CMOSImage_recording.setImage(self.currScan.CMOSImage[0,:,:])
        spots = [{'pos': pos} for pos in np.array([self.currScan.LaserPos])]
        self.CMOSScatter_recording.setData(spots)
        self.EMCCDImage_recording.setImage(self.currScan.EMCCDDisplay[0,:,:].transpose((1,0)))

        self.specSeriesImage_recording.setImage(self.currScan.RawSpecList)
        xdata = np.arange(len(self.currScan.RawSpecList[0]))
        ydata = np.array(self.currScan.RawSpecList[0])
        self.singleSpectrumItem_recording.setData(x=xdata, y=ydata)
        fitSpect = self.currScan.FitSpecList[0]
        if len(fitSpect[~np.isnan(fitSpect)]) == len(self.currScan.RawSpecList[0]):
            ydata = np.array(fitSpect)
            self.singleSpectrumItem2_recording.setData(x=np.copy(xdata), y=ydata)
        else:
            ydata = np.zeros(self.currScan.RawSpecList[0].shape)
            self.singleSpectrumItem2_recording.setData(x=np.copy(xdata), y=ydata)
        BSlist = self.currScan.BSList
        xdata = np.linspace(0,len(BSlist)-1,len(BSlist))
        xdataClean = xdata[~np.isnan(BSlist)]
        BSlistClean = BSlist[~np.isnan(BSlist)]
        self.scanDepthItem_recording.setData(x=xdataClean, y=BSlistClean)
        strIdx = self.currScan.StromaIdx
        if len(strIdx)>0:
            xdata = np.copy(xdata[strIdx])
            BSlist = np.copy(BSlist[strIdx])
            xdataClean = xdata[~np.isnan(BSlist)]
            BSlistClean = BSlist[~np.isnan(BSlist)]
            self.scanDepthItem2_recording.setData(x=xdataClean, y=BSlistClean)
        else:
            self.scanDepthItem2_recording.clear()


    def onDataViewSliderChange(self, value):
        if not self.validDataView:
            return
        self.scanDepthItem_recording.clear()
        self.dataViewInfoDisplay.setText('Frame {}/{}'.format(value, self.dataViewSlider.maximum()) )
        self.CMOSImage_recording.setImage(self.currScan.CMOSImage[value,:,:])
        spots = [{'pos': pos} for pos in np.array([self.currScan.LaserPos])]
        self.CMOSScatter_recording.setData(spots)
        self.EMCCDImage_recording.setImage(self.currScan.EMCCDDisplay[value,:,:].transpose((1,0)))
        specSeriesPartial = np.copy(self.currScan.RawSpecList)
        specSeriesPartial[value:] = 0*specSeriesPartial[value:]
        self.specSeriesImage_recording.setImage(specSeriesPartial)
        xdata = np.arange(len(self.currScan.RawSpecList[value]))
        ydata = np.array(self.currScan.RawSpecList[value])
        self.singleSpectrumItem_recording.setData(x=xdata, y=ydata)
        fitSpect = self.currScan.FitSpecList[value]
        if len(fitSpect[~np.isnan(fitSpect)]) == len(self.currScan.RawSpecList[value]):
            ydata = np.array(fitSpect)
            self.singleSpectrumItem2_recording.setData(x=np.copy(xdata), y=ydata)
        else:
            ydata = np.zeros(self.currScan.RawSpecList[value].shape)
            self.singleSpectrumItem2_recording.setData(x=np.copy(xdata), y=ydata)
        BSlist = self.currScan.BSList
        self.scanDepthPlot_recording.setXRange(0,len(BSlist)-1)
        BSlist = BSlist[:value]
        xdata = np.linspace(0,len(BSlist)-1,len(BSlist))
        xdataClean = np.copy(xdata[~np.isnan(BSlist)])
        BSlistClean = BSlist[~np.isnan(BSlist)]
        self.scanDepthItem_recording.setData(x=xdataClean, y=BSlistClean)
        strIdx = self.currScan.StromaIdx
        if len(strIdx)>0:
            idxStart = np.amin(strIdx)
            idxEnd = np.amax(strIdx)
            if value<idxStart:
                self.scanDepthItem2_recording.clear()
            elif value>idxStart and value<idxEnd:
                xdata = np.copy(xdata[strIdx[strIdx<value]])
                BSlist = np.copy(BSlist[strIdx[strIdx<value]])
                xdataClean = xdata[~np.isnan(BSlist)]
                BSlistClean = BSlist[~np.isnan(BSlist)]
                self.scanDepthItem2_recording.setData(x=xdataClean, y=BSlistClean)
            elif value>idxEnd:
                xdata = np.copy(xdata[strIdx])
                BSlist = np.copy(BSlist[strIdx])
                xdataClean = xdata[~np.isnan(BSlist)]
                BSlistClean = BSlist[~np.isnan(BSlist)]
                self.scanDepthItem2_recording.setData(x=xdataClean, y=BSlistClean)
        else:
            self.scanDepthItem2_recording.clear()
        return

    def UpdateViewerSpectrum(self, expIdx, scanIdx, specIdx):
        pass
        # rawSpect = curveData[0]
        # l = len(rawSpect)
        # xdata = np.arange(len(rawSpect))
        # ydata = np.array(rawSpect)
        # self.singleSpectrumItem.setData(x=xdata, y=ydata)

        # if self.specSeriesData.shape[1]!=len(rawSpect):    #if size of spectrum change, reset the data
        #     print('[UpdateSpectrum] Resizing spectrograph')
        #     self.specSeriesData = np.zeros((self.maxScanPoints, len(curveData)))
        #     self.specSeriesSize = 0

        # if (self.specSeriesSize < self.maxScanPoints):
        #     self.specSeriesData[self.specSeriesSize,:] = rawSpect
        #     self.specSeriesSize += 1
        # else:
        #     self.specSeriesData = np.roll(self.specSeriesData, -1, axis=0)
        #     self.specSeriesData[-1, :] = rawSpect

        # maximum = self.specSeriesData.max()
        # specSeriesDataScaled = self.specSeriesData * (255.0 / maximum)
        # self.specSeriesImage.setImage(specSeriesDataScaled)        
    #############################################################################################
    # This next group of methods are for experiment/data management (treeView)                  #
    #############################################################################################
    #TODO: save to file when renaming

    def createTreeMenu(self, position):
        menu = QtGui.QMenu()
        modelIndex = self.treeView.indexAt(position)
        item = self.model.itemFromIndex(modelIndex)

        if item is not None:
            lvl = 0
            k = item.parent()
            while k is not None:
                k = k.parent()
                lvl += 1

            if lvl == 0:
                setActiveAction = menu.addAction("Set as Active")
                action = menu.exec_(self.treeView.mapToGlobal(position))
                if action == setActiveAction:
                    self.model.setActiveExperiment(modelIndex.row())
            elif lvl == 1:
                if (item.parent().child(item.row(), 0).data(role=UserItemRoles.IsDeletedRole).toBool() == False):
                    deleteText = "Delete Scan"
                else:
                    deleteText = "Undelete Scan"
                deleteAction = menu.addAction(deleteText)
                action = menu.exec_(self.treeView.mapToGlobal(position))
                if action == deleteAction:
                    self.model.deleteScan(item)
        else:
            newAction = menu.addAction("New Experiment")
            action = menu.exec_(self.treeView.mapToGlobal(position))
            if action == newAction:
                self.session.addNewExperiment()


    def treeSelectionChanged(self, selected, deselected):
        idx = selected.indexes()[0]
        item = self.model.itemFromIndex(idx)
        if item.parent() is None:
            # print("Experiment selected")
            return
        else:
            self.updateHeatmapBySelection(idx)

            # completely refresh dataViewer tab if a different scan is selected
            if (self.dataViewerTab.currentIndex() == 1):    #dataviewer tab
                self.dataViewerTabChanged(1)

         
    def mainUI(self):

        self.validDataView = False
        self.dataViewSlider.setSingleStep(1)
        self.dataViewSlider.setPageStep(1)

        self.newSessionBtn.clicked.connect(self.createNewSession)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    GUI = App()
    GUI.show()
    sys.exit(app.exec_())

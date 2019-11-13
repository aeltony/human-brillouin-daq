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
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QTimer
import pyqtgraph as pg
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType
from pyqtgraphCustomize import *
import qt_ui # UI import
from ctypes import *
from ConfigParser import SafeConfigParser
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
        self.configParser = SafeConfigParser()
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
        spectCenter = self.configParser.getint('Andor', 'spectCenter')
        slineIdx = self.configParser.getint('Andor', 'slineIdx')
        pupilRadius = self.configParser.getfloat('Mako', 'pupilRadius')
        

        self.params = [
            {'name': 'Scan', 'type': 'group', 'children': [
                {'name': 'Start Position', 'type': 'float', 'value': -500, 'suffix':' um', 'step': 100, 'limits': (-5000, 5000)},
                {'name': 'Step size', 'type': 'float', 'value': 30, 'suffix':' um', 'step': 1, 'limits': (0, 1000), 'decimals':5},
                {'name': 'Frame number', 'type': 'int', 'value': 40, 'step': 1, 'limits':(1, 2000)},
                {'name': 'End Position', 'type': 'float', 'value': 1200, 'suffix':' um', 'readonly': True, 'decimals':5},
                {'name': 'Ref temperature', 'type': 'float', 'value': 0.0, 'suffix':' deg. C', 'readonly':True, 'decimals':4},
                {'name': 'Ref FSR', 'type': 'float', 'value':16.25, 'suffix':' GHz', 'limits':(5, 100), 'decimals':6}, 
                {'name': 'Ref SD', 'type': 'float', 'value':0.1, 'suffix':' GHz/px', 'limits':(0, 2), 'decimals':4}, 
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
                {'name': 'Jog Step', 'type': 'float', 'value': 10, 'suffix':' um', 'step': 1, 'limits':(0.1, 500)},
                {'name': 'Current Location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':5}, 
                {'name': 'Move To Location', 'type': 'float', 'value':0, 'suffix':' um', 'limits':(-1000, 2000), 'decimals':5},
                {'name': 'Jog', 'type': 'action2', 'ButtonText':('Jog Forward', 'Jog Backward')},
                {'name': 'Move/Home', 'type':'action2', 'ButtonText':('Move', 'Home')}
            ]},
            {'name': 'EMCCD', 'type': 'group', 'children': [
                {'name': 'AutoExposure', 'type':'toggle', 'ButtonText':('Auto exposure', 'Fixed exposure')},         #False=Fixed exposure
                {'name': 'Desired Temperature', 'type': 'float', 'value': -120, 'suffix':' C', 'step': 1, 'limits': (-120, 30)},
                {'name': 'Current Temperature', 'type': 'float', 'value':0, 'suffix':' C', 'readonly': True}, 
                {'name': 'EM Gain', 'type': 'int', 'value': 300, 'limits':(1, 500)},
                {'name': 'Exposure', 'type':'float', 'value':0.3, 'suffix':' s', 'step':0.05, 'limits':(0.01, 10)},
                {'name': 'Ref. Exposure', 'type':'float', 'value':1.0, 'suffix':' s', 'step':0.05, 'limits':(0.01, 10)},
                {'name': 'Spectrum Column', 'type':'int', 'value': spectCenter, 'suffix':' px', 'step':1, 'limits':(0, 512)},
                {'name': 'Spectrum Row', 'type':'int', 'value': slineIdx, 'suffix':' px', 'step':1, 'limits':(0, 64)},
            ]},        
            {'name': 'Pupil Camera', 'type': 'group', 'children': [
                {'name': 'Pupil Radius', 'type': 'float', 'value': pupilRadius, 'suffix':' px', 'step': 5, 'limits': (1, 1000)},
                {'name': 'Scale factor', 'type': 'float', 'value': 0.02, 'suffix':' (mm/px)', 'step': 0.001, 'limits': (0, 1)},
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
        self.allParameters.child('EMCCD').child('Spectrum Column').sigValueChanged.connect(
            self.spectCenterValueChange)
        self.allParameters.child('EMCCD').child('Spectrum Row').sigValueChanged.connect(
            self.slineIdxValueChange)
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
        self.TempSensorProcessThread.updateTempSeqSig.connect(self.UpdateRefTemp)
        self.TempSensorDeviceThread.start()
        self.TempSensorProcessThread.start()

        self.ShutterDevice = ShutterDevice(self)    # Initialized to SAMPLE_STATE

        self.MakoDeviceThread = MakoDevice(self.stop_event, self)
        self.MakoProcessThread = MakoFreerun(self.MakoDeviceThread, self.stop_event)
        self.MakoProcessThread.updateCMOSImageSig.connect(self.MakoProcessUpdate)
        self.MakoDeviceThread.start()
        self.MakoProcessThread.start()

        self.AndorDeviceThread = AndorDevice(self.stop_event, self)
        self.AndorProcessThread = AndorProcessFreerun(self.AndorDeviceThread, self.stop_event)
        self.AndorProcessThread.updateEMCCDImageSig.connect(self.AndorProcessUpdate)

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
        scaleFactor = self.allParameters.child('Pupil Camera').child('Scale factor').value()
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

        # Andor Live image
        self.EMCCDview = CustomViewBox(invertY=True)     
        self.graphicsViewEMCCD.setCentralItem(self.EMCCDview)    
        self.EMCCDview.setAspectLocked(True)
        self.EMCCDImage = pg.ImageItem(np.zeros((1024, 170)))                        
        self.EMCCDview.addItem(self.EMCCDImage)
        self.EMCCDview.autoRange(padding=0)

        # Andor recorded image
        self.EMCCDview_recording = CustomViewBox(invertY=True)                      
        self.dataViewEMCCD.setCentralItem(self.EMCCDview_recording)    
        self.EMCCDview_recording.setAspectLocked(True)
        self.EMCCDImage_recording = pg.ImageItem(np.zeros((1024, 170)))                         
        self.EMCCDview_recording.addItem(self.EMCCDImage_recording)
        self.EMCCDview_recording.autoRange(padding=0)

        self.maxScanPoints = 400  # Number of data points in view in freerunning mode

        # singleSpectrumPlot is the plot of the raw data and Lorentzian fit
        self.singleSpectrumPlot = pg.PlotItem()
        # self.singleSpectrumPlot.setYRange(0, 17500)
        self.singleSpectrumPlot.enableAutoRange(axis=self.singleSpectrumPlot.vb.YAxis, enable=True)
        self.graphicsViewSingleSpectrum.setCentralItem(self.singleSpectrumPlot)
        self.singleSpectrumItem = pg.PlotDataItem(symbol='o')
        self.singleSpectrumItem.setSymbolPen(color='g')
        self.singleSpectrumItem.setPen(None)
        self.singleSpectrumItem.setData(100*np.ones(100))
        self.singleSpectrumItem2 = pg.PlotDataItem()
        self.singleSpectrumItem2.setPen(width=2.5, color='r')
        self.singleSpectrumItem2.setData(200*np.ones(100))
        self.singleSpectrumPlot.addItem(self.singleSpectrumItem)
        self.singleSpectrumPlot.addItem(self.singleSpectrumItem2)
        axBottom = self.singleSpectrumPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.singleSpectrumPlot.getAxis('left')
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

        # scanDepthPlot is the Brillouin vs z axis plot
        self.scanDepthPlot = pg.PlotItem()
        self.scanDepthPlot.setYRange(5,6)
        self.graphicsViewScanDepth.setCentralItem(self.scanDepthPlot)
        self.scanDepthPlot.enableAutoRange(axis=self.scanDepthPlot.vb.XAxis, enable=True)
        self.scanDepthItem = pg.PlotDataItem() 
        self.scanDepthItem.setPen(width=2.5, color='g')
        self.scanDepthItem2 = pg.PlotDataItem()
        self.scanDepthItem2.setPen(width=2.5, color='b')
        self.scanDepthPlot.addItem(self.scanDepthItem)
        self.scanDepthPlot.addItem(self.scanDepthItem2)
        axBottom = self.scanDepthPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.scanDepthPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')
        self.scanDepthData = np.array([])
        self.scanDepthData2 = np.array([])

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

        self.specSeriesData = np.zeros((self.maxScanPoints, 512))
        self.specSeriesSize = 0
        self.specSeriesPlot = pg.PlotItem()
        self.graphicsViewSpecSeries.setCentralItem(self.specSeriesPlot)
        self.specSeriesImage = pg.ImageItem(self.specSeriesData)     
        self.specSeriesPlot.addItem(self.specSeriesImage)
        axBottom = self.specSeriesPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.specSeriesPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        self.mainUI()

        self.MakoProcessThread.pupilRadius = self.allParameters.child('Pupil Camera').child('Pupil Radius').value()
        self.AndorProcessThread.spectCenter = self.allParameters.child('EMCCD').child('Spectrum Column').value()
        self.AndorProcessThread.slineIdx = self.allParameters.child('EMCCD').child('Spectrum Row').value()
        self.AndorDeviceThread.start()
        self.AndorDeviceThread.setPriority(QtCore.QThread.TimeCriticalPriority)
        self.AndorProcessThread.start()
        time.sleep(0.1)   # CRUCIAL!! DON'T REMOVE. Need to let the threads fully start before continuing

        # create the figures for plotting Brillouin shifts and fits
        self.AndorProcessThread.updateBrillouinSeqSig.connect(self.UpdateBrillouinSeqPlot)
        self.AndorProcessThread.updateSpectrum.connect(self.UpdateSpectrum)

        self.BrillouinScan = ScanManager(self.cancel_event, self.stop_event, self.ZaberDevice, self.ShutterDevice)
        self.BrillouinScan.addToSequentialList(self.AndorDeviceThread, self.AndorProcessThread)
        self.BrillouinScan.addToSequentialList(self.MakoDeviceThread, self.MakoProcessThread)
        self.BrillouinScan.addToSequentialList(self.TempSensorDeviceThread, self.TempSensorProcessThread)
        self.BrillouinScan.finished.connect(self.onFinishScan)
        self.BrillouinScan.clearGUISig.connect(self.clearGUIElements)

        self.InitHardwareParameterTree()

    def drawCircle(self, radius, center=(0.0,0.0)):
        angle = np.linspace(0,2*math.pi,100)
        circleX = radius*np.cos(angle) + center[0]
        circleY = radius*np.sin(angle) + center[1]
        return (circleX, circleY)

    def CMOShLineValueChange(self, param, value):
        # print "[CMOShLineValueChange]"
        self.CMOShLine.setPos(value)
        self.configParser.set('Scan', 'laser_position_Y', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def CMOSvLineValueChange(self, param, value):
        # print "[CMOSvLineValueChange]"
        self.CMOSvLine.setPos(value)
        self.configParser.set('Scan', 'laser_position_X', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def spectCenterValueChange(self, param, value):
        # print "[spectCenterValueChange]"
        self.AndorProcessThread.spectCenter = self.allParameters.child('EMCCD').child('Spectrum Column').value()
        self.configParser.set('Andor', 'spectCenter', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def slineIdxValueChange(self, param, value):
        self.AndorProcessThread.slineIdx = self.allParameters.child('EMCCD').child('Spectrum Row').value()
        self.configParser.set('Andor', 'slineIdx', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def pupilRadiusValueChange(self, param, value):
        # print "[pupilRadiusValueChange]"
        self.MakoProcessThread.pupilRadius = self.allParameters.child('Pupil Camera').child('Pupil Radius').value()
        self.configParser.set('Mako', 'pupilRadius', str(float(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    #############################################################################################
    # This next group of methods are used to set/get hardware settings, using the parameterTree #
    #############################################################################################
    def InitHardwareParameterTree(self):
        # print "[InitHardwareParameterTree]"

        # ========================= EMCCD ================================
        pItem = self.allParameters.child('EMCCD')
        pItem.child('AutoExposure').sigActivated.connect(self.switchAutoExp)
        pItem.child('Desired Temperature').sigValueChanged. \
            connect(lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setTemperature))
        pItem.child('Current Temperature').setValue(self.AndorDeviceThread.getTemperature())
        pItem.child('EM Gain').setValue(self.AndorDeviceThread.getEMCCDGain())
        pItem.child('EM Gain').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setEMCCDGain))
        pItem.child('Exposure').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setExposure))
        pItem.child('Exposure').setValue(self.AndorDeviceThread.getExposure())

        # ========================= Motor =================================
        pItem = self.allParameters.child('Motor')
        self.MotorPositionUpdate()

        # connect Jog Buttons
        motorFwdFun = lambda: self.ZaberDevice.moveRelative( 
            self.allParameters.child('Motor').child('Jog Step').value() )
        motorBackFun = lambda: self.ZaberDevice.moveRelative( 
            -self.allParameters.child('Motor').child('Jog Step').value() )
        pItem.child('Jog').sigActivated.connect(motorFwdFun)
        pItem.child('Jog').sigActivated2.connect(motorBackFun)

        # connect Move and Home button
        motorMoveFun = lambda: self.ZaberDevice.setMotorAsync(
            'moveAbs', [self.allParameters.child('Motor').child('Move To Location').value()])
        # motorMoveFun = lambda: self.ZaberDevice.moveAbs( 
        #     self.allParameters.child('Motor').child('Move To Location').value() )
        pItem.child('Move/Home').sigActivated.connect(motorMoveFun)
        pItem.child('Move/Home').sigActivated2.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome'))

        # ========================= Scan ===================
        pItem = self.allParameters.child('Scan')
        startPos = pItem.child('Start Position').value()
        stepSize = pItem.child('Step size').value()
        frameNum = pItem.child('Frame number').value()
        endPos = startPos + stepSize * frameNum
        pItem.child('End Position').setValue(endPos)
        pItem.child('Scan/Cancel').sigActivated.connect(self.startScan)
        pItem.child('Scan/Cancel').sigActivated2.connect(self.cancelScan)
        pItem.child('Start Position').sigValueChanging.connect(lambda param, value: self.updateScanEndPos(0, param, value))
        pItem.child('Step size').sigValueChanging.connect(lambda param, value: self.updateScanEndPos(1, param, value))
        pItem.child('Frame number').sigValueChanging.connect(lambda param, value: self.updateScanEndPos(2, param, value))

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
        # print "[changeHardwareSetting]"
        funcHandle(data.value())

    def HardwareParamUpdate(self):
        # print "[HardwareParamUpdate]"
        temp = self.AndorDeviceThread.getTemperature()
        self.allParameters.child('EMCCD').child('Current Temperature').setValue(temp)
        # if (self.ShutterDevice.state == ShutterDevice.SAMPLE_STATE):
            # expTime = self.AndorDeviceThread.getExposure()
            # self.allParameters.child('EMCCD').child('Exposure').setValue(expTime)

    def updateScanEndPos(self, updateIndex, param, value):
        # print "[updateScanEndPos]"
        pItem = self.allParameters.child('Scan')
        startPos = pItem.child('Start Position').value()
        stepSize = pItem.child('Step size').value()
        frameNum = pItem.child('Frame number').value()
        if (updateIndex == 0):
            startPos = value
        elif (updateIndex == 1):
            stepSize = value
        else:
            frameNum = value
        endPos = startPos + stepSize * (frameNum - 1)
        pItem.child('End Position').setValue(endPos)

    @QtCore.pyqtSlot(float)
    def MotorPositionUpdate2(self, pos):
        # print "[MotorPositionUpdate2]"
        self.allParameters.child('Motor').child('Current Location').setValue(pos)

    def MotorPositionUpdate(self):
        # print "[MotorPositionUpdate]"
        pos = self.ZaberDevice.getCurrentPosition()
        self.allParameters.child('Motor').child('Current Location').setValue(pos)

    @QtCore.pyqtSlot()
    def clearGUIElements(self):
        # print "[clearGUIElements]"
        self.specSeriesData = np.zeros((self.maxScanPoints, 512))
        self.specSeriesSize = 0
        self.scanDepthData = np.array([])
        self.scanDepthData2 = np.array([])

    def startScan(self):
        # First check that a session is running, and that an experiment is selected
        if self.session is None:
            choice = QtGui.QMessageBox.warning(self, 'Starting Scan...',
                                                "No Session open!",
                                                QtGui.QMessageBox.Ok)
            return

        print "Starting a scan in Exp_%d: " % self.model.activeExperiment

        # take screenshot
        p = QtGui.QPixmap.grabWindow(self.winId())
        pImage = p.toImage()
        channels = 4
        s = pImage.bits().asstring(p.width() * p.height() * channels)
        screenshotArr = np.fromstring(s, dtype=np.uint8).reshape((p.height(), p.width(), channels))


        flattenedParamList = generateParameterList(self.params, self.allParameters)

        scanSettings = {'start': self.allParameters.child('Scan').child('Start Position').value(),  
            'step': self.allParameters.child('Scan').child('Step size').value(),   
            'frames': self.allParameters.child('Scan').child('Frame number').value(),
            'laserX': self.allParameters.child('Scan').child('More Settings').child('Laser Focus X').value(),
            'laserY': self.allParameters.child('Scan').child('More Settings').child('Laser Focus Y').value(),
            'scaleFactor': self.allParameters.child('Pupil Camera').child('Scale factor').value(),
            'refExp': self.allParameters.child('EMCCD').child('Ref. Exposure').value(),
            'sampleExp': self.allParameters.child('EMCCD').child('Exposure').value(),
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
        print 'Stopping current scan and wrapping-up.'
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
        print 'Scan completed'

    def toggleReference(self, sliderParam, state):
        # print "[toggleReference]"
        # state == True --> Reference
        # state == False --> sample
        if state:
            self.ShutterDevice.setShutterState(self.ShutterDevice.REFERENCE_STATE)
            self.AndorDeviceThread.setExposure(self.allParameters.child('EMCCD').child('Ref. Exposure').value())
        else:
            self.ShutterDevice.setShutterState(self.ShutterDevice.SAMPLE_STATE)
            self.AndorDeviceThread.setExposure(self.allParameters.child('EMCCD').child('Exposure').value())

    def switchAutoExp(self, sliderParam, state):
        if state:
            self.AndorDeviceThread.setAutoExp(True)
            print "EMCCD auto exposure ON"
        else:
            self.AndorDeviceThread.setAutoExp(False)
            self.AndorDeviceThread.setExposure(self.allParameters.child('EMCCD').child('Exposure').value())
            print "EMCCD auto exposure OFF"
            # self.AndorDeviceThread.setExposure(self.allParameters.child('EMCCD').child('Exposure').value())

    #############################################################################################
    # This next group of methods callback methods to display acquired data #
    #############################################################################################

    def updateHeatmapByExp(self, expIdx):
        # print "[updateHeatmapByExp]"
        currExp = self.session.experimentList[expIdx]
        laserCoords = np.array([np.float(self.allParameters.child('Scan').child('More Settings').child('Laser Focus X').value()), \
            np.float(self.allParameters.child('Scan').child('More Settings').child('Laser Focus Y').value())])
        coords = currExp.getMeanScanCoords(laserCoords)
        self.makoPoints = coords
        BS = currExp.getBS()
        activeScanIdx = currExp.getActiveScanIndices()
        self.updateHeatmap(coords, BS, activeScanIdx, expIdx)

    def updateHeatmapBySelection(self, idx):
        # print "[updateHeatmapBySelection]"
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
            print p.data()

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
        # print "expIdx = %d, scanIdx = %d" % (expIdx, scanIdx)
        p = None
        for p1 in heatmapScatter.points():
            if p1.data()[0] == expIdx and p1.data()[1] == scanIdx:
                p = p1
                continue
        if p is None:   # point not found if scan is deleted
            return

        # highlight the found point
        # print "pen found"
        p.setPen('w', width=2)
        lastClicked[:] = [p]    # only allow single selection   


    def updateHeatmap(self, scanPoints, BS, activeScanIndices, expIndex):
        # print "[updateHeatmap]"
        if self.dataViewerTab.currentIndex() == 0:  #acq tab
            lastClicked = self.heatmapScatterLastClicked
            heatmapImage = self.heatmapImage
        else:   #viewer tab
            lastClicked = self.heatmapScatterLastClicked_recording
            heatmapImage = self.heatmapImage_recording

        scaleFactor = self.allParameters.child('Pupil Camera').child('Scale factor').value()
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
                print '[updateHeatmap] Could not generate heatmap'
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
        # print "[heatmapScatterplotClicked]"
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

    def UpdateRefTemp(self, temperature):
        # print 'UpdateRefTemp'
        self.allParameters.child('Scan').child('Ref temperature').setValue(temperature)

    # updates the figure containing the Brillouin sequence. newData is a list
    def UpdateBrillouinSeqPlot(self, interPeakDist):
        # print "[UpdateBrillouinSeqPlot]"
        SD = self.allParameters.child('Scan').child('Ref SD').value()
        FSR = self.allParameters.child('Scan').child('Ref FSR').value()
        T = self.allParameters.child('Scan').child('Ref temperature').value()

        if len(interPeakDist)==2:
            newData = [0.5*(FSR - SD*interPeakDist[1])]
            newData2 = newData
        elif len(interPeakDist)==3:
            ## CALIBRATION if reference arm open and signal counts in optimal range
            if (self.ShutterDevice.state == ShutterDevice.REFERENCE_STATE and interPeakDist[0]<15900 and interPeakDist[0]>10000):
                print "Updating calibration..."
                WaterBS = self.waterConst[0]*T*T + self.waterConst[1]*T + self.waterConst[2]
                PlasticBS = 16.3291 - (self.plasticConst[0]*T*T + self.plasticConst[1]*T + self.plasticConst[2])
                SD = 2*(PlasticBS - WaterBS)/(interPeakDist[1] + interPeakDist[2])
                FSR = 2*WaterBS + interPeakDist[1]*SD
                self.allParameters.child('Scan').child('Ref SD').setValue(SD)
                self.allParameters.child('Scan').child('Ref FSR').setValue(FSR)
            newData = [0.5*(FSR - SD*interPeakDist[1])]
            newData2 = [0.5*(FSR - SD*interPeakDist[2])]
        else:
            newData = [np.nan]
            newData2 = [np.nan]

        if len(newData)+len(self.scanDepthData) > self.maxScanPoints:
            t = self.maxScanPoints - len(newData) - len(self.scanDepthData)
            self.scanDepthData = np.roll(self.scanDepthData, t)
            self.scanDepthData[self.maxScanPoints - len(newData):] = newData
        else:
            self.scanDepthData = np.append(self.scanDepthData, newData)

        xdata = np.arange(len(self.scanDepthData))
        xdata = xdata[~np.isnan(self.scanDepthData)]
        ydata = self.scanDepthData[~np.isnan(self.scanDepthData)]
        self.scanDepthItem.setData(x=xdata, y=ydata)

        if len(newData2)+len(self.scanDepthData2) > self.maxScanPoints:
            t = self.maxScanPoints - len(newData2) - len(self.scanDepthData2)
            self.scanDepthData2 = np.roll(self.scanDepthData2, t)
            self.scanDepthData2[self.maxScanPoints - len(newData2):] = newData2
        else:
            self.scanDepthData2 = np.append(self.scanDepthData2, newData2)
        xdata2 = np.arange(len(self.scanDepthData2))
        xdata2 = xdata2[~np.isnan(self.scanDepthData2)]
        ydata2 = self.scanDepthData2[~np.isnan(self.scanDepthData2)]
        self.scanDepthItem2.setData(x=xdata2, y=ydata2)

    # Plot fitted Brillouin spectrum
    # curvedata is a tuple of (raw spectrum, fitted spectrum)
    def UpdateSpectrum(self,curveData):
        # print "[UpdateSpectrum]"
        rawSpect = curveData[0]
        fitSpect = curveData[1]

        xdata = np.arange(len(rawSpect))

        self.singleSpectrumItem.setData(x=xdata, y=np.array(rawSpect))
        if len(fitSpect[~np.isnan(fitSpect)]) == len(rawSpect):
            self.singleSpectrumItem2.setData(x=np.copy(xdata), y=np.array(fitSpect))
        else:
            self.singleSpectrumItem2.setData(x=np.copy(xdata), y=np.zeros(len(rawSpect)))

        if self.specSeriesData.shape[1]!=len(rawSpect):    #if size of spectrum change, reset the data
            # print '[UpdateSpectrum] Resizing spectrograph'
            self.specSeriesData = np.zeros((self.maxScanPoints, len(rawSpect)))
            self.specSeriesSize = 0

        if (self.specSeriesSize < self.maxScanPoints):
            self.specSeriesData[self.specSeriesSize,:] = rawSpect
            self.specSeriesSize += 1
        else:
            self.specSeriesData = np.roll(self.specSeriesData, -1, axis=0)
            self.specSeriesData[-1, :] = rawSpect

        maximum = self.specSeriesData.max()
        specSeriesDataScaled = self.specSeriesData #* (255.0 / maximum)
        self.specSeriesImage.setImage(specSeriesDataScaled)


    # Update raw Andor image
    def AndorProcessUpdate(self, image):
        # print "[AndorProcessUpdate]"
        #(counter, image) = self.AndorProcessThread.processedData.get()
        # img_rect = QtCore.QRectF(0, 0, 1024, 170)
        # self.EMCCDImage.setRect(img_rect)
        self.EMCCDImage.setImage(image.transpose((1,0)))

    # Update the CMOS camera image
    # makoData[0] is the CMOS camera image with pupil detection
    # makoData[1] is a tuple with the pupil center coordinate
    def MakoProcessUpdate(self, makoData):
        # print "[MakoProcessUpdate]"
        image = makoData[0]
        center = np.array([makoData[1]])
        # center[1] = self.MakoDeviceThread.imageHeight - center[1]
        # (counter, data) = self.MakoProcessThread.processedData.get()
        # print 'self.makoPoints =', self.makoPoints
        # print 'center =', center
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
        # print filename

        # if filename:
        #     filename = str(filename)
        #     fileBase, fileExtension = os.path.splitext(str(filename))
        #     if (fileExtension.lower() != 'hdf5'.lower()):
        #         filename = filename + '.hdf5'
        # else:
        #     return

        self.dataFileName = filename
        self.sessionName.setText(QtCore.QString(filename))

        #create a single new session
        self.session = SessionData(ntpath.basename(self.dataFileName), filename=filename)
        self.model.session = self.session

        self.session.updateTreeViewSig.connect(
            lambda updateIndices:self.model.updateTree(updateIndices, self.treeView))
        self.session.addNewExperiment()

        self.treeView.setEnabled(True)


    def closeEvent(self,event):
        print "Program Shutdown"
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

        event.accept() #closes the application

    #############################################################################################
    # This next group of methods are for dataViewer widgets                                     #
    #############################################################################################
    def dataViewerTabChanged(self, index):
        if index == 0:
            # in data acq tab, disconnect data view tab widget signals
            if self.validDataView:
                self.dataViewSlider.valueChanged.disconnect()   

        print "[Data Viewer Tab] Open"
        

        # Find which dataset is selected
        allSelected = self.treeView.selectionModel().selectedIndexes()
        if len(allSelected)>0:
            lastSelected = allSelected[0]
            # self.updateHeatmapBySelection(lastSelected)
            item = self.model.itemFromIndex(lastSelected)

            if item.parent() is None:       # Experiment selected (not scan)
                expIdx = item.row()
                if (item.rowCount() == 0):  # No scan in experiment
                    print "[dataViewerTabChanged] no scan in experiment"
                    return
                scanIdx = 0
            else:
                expIdx = item.parent().row()
                scanIdx = item.row()        
                print "[dataViewerTabChanged] Selected exp_%d scan_%d" % (expIdx, scanIdx)
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
        #     print '[UpdateSpectrum] Resizing spectrograph'
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
            # print "Experiment selected"
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

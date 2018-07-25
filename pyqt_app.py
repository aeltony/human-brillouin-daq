
import threading
import datetime
import imutils
import cv2
import numpy as np
import time
import math
import os
import sys 
import traceback

from PyQt4 import QtGui,QtCore
from PIL import Image
from PIL import ImageTk
import hough_transform as ht
import skvideo.io as skv

# device imports
import device_init
from pymba import *
from my_andor.andor_wrap import *
from ctypes import *
import zaber.serial as zs

# graphing imports
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from scipy.optimize import curve_fit

class App(QtGui.QWidget):

    radius_signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self):
        super(App,self).__init__()

        #Lock used to halt other threads upon app closing
        self.stop_event = threading.Event()
        self.condition = threading.Condition()

        # initialize and access cameras, motors and graphs
        self.mako = device_init.Mako_Camera()
        #self.andor = device_init.Andor_Camera()
        self.motor = device_init.Motor()
        self.graph = Graph()

        self.brillouin_shift_list = []
        self.PlasticBS =  9.6051
        self.WaterBS = 5.1157
        self.shutter_state = False

        self.subplot = None
        self.brillouin_plot = None
        self.FSR = None
        self.SD = None

        self.pupil_video_frames = []
        self.pupil_data_list = []
        self.andor_image_list = []
        self.scan_ready = False
        self.andor_export_image = None
        self.expected_pupil_radius = None
        self.popup = None

        self.CMOSthread = CMOSthread(self)
        self.connect(self.CMOSthread,QtCore.SIGNAL('update_CMOS_panel(PyQt_PyObject)'),self.update_CMOS_panel)

        #self.EMCCDthread = EMCCDthread(self.andor,self.motor,self.graph)
        #self.EMCCDthread.camera_signal.connect(self.update_EMCCD_panel)
        #self.EMCCDthread.graph_signal.connect(self.update_graph_panel)
        #self.EMCCDthread.curve_signal.connect(self.draw_curve)

        ### CMOS AND EMCCD PANEL ###
        self.cmos_panel = QtGui.QLabel()
        self.emccd_panel = QtGui.QLabel()
        self.canvas = FigureCanvasQTAgg(self.graph.fig)

        self.mainUI()
        #self.EMCCDthread.set_entries(self.FSR_entry,self.SD_entry)

        self.CMOSthread.start()
        #self.EMCCDthread.start()

    def mainUI(self):
        grid = QtGui.QGridLayout()
        self.setLayout(grid)


        grid.addWidget(self.cmos_panel,0,2,11,7)
        grid.addWidget(self.emccd_panel,0,9,3,5)
        grid.addWidget(self.canvas,1,9,3,5)

        self.cmos_panel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
        
        #############################
        ### PUPIL DETECTION PANEL ###
        #############################

        det_grid = QtGui.QGridLayout()
        grid.addLayout(det_grid,0,0,11,2)

        detection_panel_label = QtGui.QLabel("Pupil Detection Panel")
        blur_label = QtGui.QLabel("medianBlur =")
        blur_entry = QtGui.QLineEdit()
        dp_label = QtGui.QLabel("dp =")
        dp_entry = QtGui.QLineEdit()
        minDist_label = QtGui.QLabel("minDist =")
        minDist_entry = QtGui.QLineEdit()
        param1_label = QtGui.QLabel("param1 =")
        param1_entry = QtGui.QLineEdit()
        param2_label = QtGui.QLabel("param2 =")
        param2_entry = QtGui.QLineEdit()
        range_label = QtGui.QLabel("range =")
        range_entry = QtGui.QLineEdit()
        radius_label = QtGui.QLabel("radius =")
        radius_entry = QtGui.QLineEdit()
        radius_btn = QtGui.QPushButton("Draw radius estimate",self)
        default_btn = QtGui.QPushButton("Restore Defaults",self)
        apply_btn = QtGui.QPushButton("Apply Changes",self)
        
        det_grid.addWidget(detection_panel_label, 0, 0, 1, 2)
        det_grid.addWidget(blur_label, 1, 0)
        det_grid.addWidget(blur_entry, 1, 1)
        det_grid.addWidget(dp_label, 2, 0)
        det_grid.addWidget(dp_entry, 2, 1)
        det_grid.addWidget(minDist_label, 3, 0)
        det_grid.addWidget(minDist_entry, 3, 1)
        det_grid.addWidget(param1_label, 4, 0)
        det_grid.addWidget(param1_entry, 4, 1)
        det_grid.addWidget(param2_label, 5, 0)
        det_grid.addWidget(param2_entry, 5, 1)
        det_grid.addWidget(range_label, 6, 0)
        det_grid.addWidget(range_entry, 6, 1)
        det_grid.addWidget(radius_label, 7, 0)
        det_grid.addWidget(radius_entry, 7, 1)
        det_grid.addWidget(radius_btn, 8, 0, 1, 2)
        det_grid.addWidget(default_btn, 9, 0, 1, 2)
        det_grid.addWidget(apply_btn, 10, 0, 1, 2)

        blur_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        dp_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        minDist_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        param1_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        param2_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        range_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        radius_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        blur_entry.setText("15")
        dp_entry.setText("3.0")
        minDist_entry.setText("1000")
        param1_entry.setText("1")
        param2_entry.setText("300")
        range_entry.setText("15")
        radius_entry.setText("180")

        self.blur_entry = blur_entry
        self.dp_entry = dp_entry
        self.minDist_entry = minDist_entry
        self.param1_entry = param1_entry
        self.param2_entry = param2_entry
        self.range_entry = range_entry
        self.radius_entry = radius_entry

        radius_btn.clicked.connect(self.CMOSthread.ask_radius_estimate)
        default_btn.clicked.connect(self.restore_default_params)
        apply_btn.clicked.connect(self.CMOSthread.apply_parameters)

        ##########################
        ### PUPIL CAMERA PANEL ###
        ##########################

        cam_grid = QtGui.QGridLayout()
        grid.addLayout(cam_grid,11,2,1,2)

        snapshot_btn = QtGui.QPushButton("Take Picture",self)
        record_btn = QtGui.QPushButton("Record",self)
        record_btn.setCheckable(True)

        cam_grid.addWidget(snapshot_btn, 0, 0)
        cam_grid.addWidget(record_btn, 0, 1)

        record_btn.clicked.connect(self.CMOSthread.trigger_record)

        self.record_btn = record_btn
        
        ###################
        ### GRAPH PANEL ###
        ###################

        graph_grid = QtGui.QGridLayout()
        grid.addLayout(graph_grid,11,4,1,5)

        reference_btn = QtGui.QPushButton("Reference",self)
        reference_btn.setCheckable(True)
        FSR_label = QtGui.QLabel("FSR")
        FSR_entry = QtGui.QLineEdit()
        SD_label = QtGui.QLabel("SD")
        SD_entry = QtGui.QLineEdit()

        graph_grid.addWidget(reference_btn, 0, 0)
        graph_grid.addWidget(FSR_label, 0, 1)
        graph_grid.addWidget(FSR_entry, 0, 2)
        graph_grid.addWidget(SD_label, 0, 3)
        graph_grid.addWidget(SD_entry, 0, 4)

        FSR_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        SD_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        FSR_entry.setText("16.2566")
        SD_entry.setText("0.14288")

        self.reference_btn = reference_btn
        self.FSR_entry = FSR_entry
        self.SD_entry = SD_entry

        self.reference_btn.clicked.connect(lambda: self.shutters())

        ###################
        ### MOTOR PANEL ###
        ###################

        motor_grid = QtGui.QGridLayout()
        grid.addLayout(motor_grid,12,2,2,7)

        motor_label = QtGui.QLabel("Motor Control")
        home_btn = QtGui.QPushButton("Home",self)
        distance_label = QtGui.QLabel("Distance To Move")
        distance_entry = QtGui.QLineEdit()
        forward_btn = QtGui.QPushButton("Forward",self)
        backward_btn = QtGui.QPushButton("Backward",self)
        location_label = QtGui.QLabel("Location")
        location_entry = QtGui.QLineEdit()
        position_btn = QtGui.QPushButton("Move To Location",self)

        motor_grid.addWidget(motor_label, 0, 0)
        motor_grid.addWidget(home_btn, 1, 0)
        motor_grid.addWidget(distance_label, 0, 1)
        motor_grid.addWidget(distance_entry, 1, 1)
        motor_grid.addWidget(forward_btn, 1, 2)
        motor_grid.addWidget(backward_btn, 1, 3)
        motor_grid.addWidget(location_label, 0, 5)
        motor_grid.addWidget(location_entry, 1, 5)
        motor_grid.addWidget(position_btn, 1, 6)

        self.distance_entry = distance_entry
        self.distance_entry.setText("0")
        self.location_entry = location_entry
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(int(loc.data*3.072)))

        home_btn.clicked.connect(self.move_motor_home)
        forward_btn.clicked.connect(lambda: self.move_motor_forward(float(self.distance_entry.displayText())))
        backward_btn.clicked.connect(lambda: self.move_motor_backward(-float(self.distance_entry.displayText())))
        position_btn.clicked.connect(lambda: self.move_motor_abs(float(self.location_entry.displayText())))

        #############################
        ### DATA COLLECTION PANEL ###
        #############################

        data_grid = QtGui.QGridLayout()
        grid.addLayout(data_grid,14,2,2,4)

        start_pos_label = QtGui.QLabel("Start Position(um)")
        start_pos = QtGui.QLineEdit()
        scan_length_label = QtGui.QLabel("Length(um)")
        scan_length = QtGui.QLineEdit()
        num_frames_label = QtGui.QLabel("#Frames")
        num_frames = QtGui.QLineEdit()
        scan_btn = QtGui.QPushButton("Start Scan",self)

        data_grid.addWidget(start_pos_label, 0, 0)
        data_grid.addWidget(start_pos, 1, 0)
        data_grid.addWidget(scan_length_label, 0, 1)
        data_grid.addWidget(scan_length, 1, 1)
        data_grid.addWidget(num_frames_label, 0, 2)
        data_grid.addWidget(num_frames, 1, 2)
        data_grid.addWidget(scan_btn, 1, 3)

        scan_btn.clicked.connect(lambda: self.EMCCDthread.scan(float(start_pos.displayText()),float(scan_length.displayText()),float(num_frames.displayText())))

        #######################################
        ### VELOCITY AND ACCELERATION PANEL ###
        #######################################

        vel_grid = QtGui.QGridLayout()
        grid.addLayout(vel_grid,14,6,2,2)

        velocity_label = QtGui.QLabel("Velocity")
        velocity = QtGui.QLineEdit()
        velocity_btn = QtGui.QPushButton("Change Velocity",self)

        vel_grid.addWidget(velocity_label, 0, 0)
        vel_grid.addWidget(velocity, 1, 0)
        vel_grid.addWidget(velocity_btn, 1, 1)


        self.setWindowTitle("Brillouin Scan Interface")
        self.move(50,50)
        self.show()
        print "finished showing"

    def update_CMOS_panel(self,qImage):
        #print "updating CMOS panel"
        pixmap = QtGui.QPixmap.fromImage(qImage)
        self.cmos_panel.setPixmap(pixmap)
        self.cmos_panel.show()

    def update_EMCCD_panel(self,qImage):
        #print "updating EMCCD panel"
        pixmap = QtGui.QPixmap.fromImage(qImage)
        self.emccd_panel.setPixmap(pixmap)
        self.emccd_panel.show()

    def update_graph_panel(self,graph_data):

        copied_analyzed_row, brillouin_shift_list = graph_data

        self.graph.fig.clf()
        self.subplot = self.graph.fig.add_subplot(211)
        self.subplot.set_xlabel("Pixel")
        self.subplot.set_ylabel("Counts")

        self.brillouin_plot = self.graph.fig.add_subplot(212)

        self.subplot.scatter(self.graph.x_axis, copied_analyzed_row, s = 1)
        #self.brillouin_plot.scatter(np.arange(1, len(brillouin_shift_list)+1), np.array(brillouin_shift_list))

        self.canvas.draw()

    def draw_curve(self,curve_data):
        if len(curve_data) == 2:
            popt, pcov = curve_data
            self.subplot.plot(self.graph.x_axis, lorentzian(self.graph.x_axis, *popt), 'r-', label='fit')
        elif len(curve_data) == 4:
            popt, pcov, measured_SD, measured_FSR = curve_data
            self.subplot.plot(self.graph.x_axis, lorentzian_reference(self.graph.x_axis, *popt), 'r-', label='fit')
            self.SD_entry.setText(measured_SD)
            self.FSR_entry.setText(measured_FSR)

        self.canvas.draw()

    #similar to shutters.py, called on by reference button 
    def shutters(self, close = False):
        dll = WinDLL("C:\\Program Files\\quad-shutter\\Quad Shutter dll and docs\\x64\\PiUsb")
        c2 = c_int()
        c4 = c_int()
        usb312 = dll.piConnectShutter(byref(c2), 312)
        usb314 = dll.piConnectShutter(byref(c4), 314)

        
        state = self.reference_btn.isChecked()
        self.EMCCDthread.set_shutter_state(state)
            
        if state and not close:
            dll.piSetShutterState(0, usb312)
            dll.piSetShutterState(1, usb314)
        else:
            dll.piSetShutterState(1, usb312)
            dll.piSetShutterState(0, usb314)

        dll.piDisconnectShutter(usb312)
        dll.piDisconnectShutter(usb314)

    # moves zaber motor to home position
    def move_motor_home(self):
        self.motor.device.home()
        loc = self.motor.device.send(60,0)
        self.location_entry.setText(str(int(loc.data*3.072)))

    # moves zaber motor, called on by forwards and backwards buttons
    def move_motor_forward(self,distance):
        self.motor.device.move_rel(int(distance/3.072))
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(int(loc.data*3.072)))

    def move_motor_backward(self,distance):
        self.motor.device.move_rel(-int(distance/3.072))
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(int(loc.data*3.072)))

     # moves zaber motor to a set location, called on above
    def move_motor_abs(self,pos):
        self.motor.device.move_abs(int(pos/3.072))
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(int(loc.data*3.072)))

    def set_velocity(self, velocity):
        self.motor.device.send(42,velocity)

    def restore_default_params(self):
        self.dp_entry.setText("3.0")
        self.minDist_entry.setText("1000")
        self.param1_entry.setText("1")
        self.param2_entry.setText("300")
        self.range_entry.setText("15")
        self.radius_entry.setText("180")

    def set_radius_entry(self,expected_pupil_radius):
        self.radius_entry.setText(str(expected_pupil_radius))

    def closeEvent(self,event):
        self.stop_event.set()
        self.mako.camera.runFeatureCommand('AcquisitionStop')
        self.mako.camera.endCapture()
        self.mako.camera.revokeAllFrames()
        self.mako.vimba.shutdown()
        self.motor.port.close()
        self.shutters(close = True)

        event.accept() #closes the application


class Popup(QtGui.QWidget):

    def __init__(self,CMOSthread):
        super(Popup,self).__init__()

        self.CMOSthread = CMOSthread
        self.qImage_snapshot = CMOSthread.qImage

        grid = QtGui.QGridLayout()
        self.setLayout(grid)
        
        #self.radius_signal = App.radius_signal

        pixmap = QtGui.QPixmap.fromImage(self.qImage_snapshot)

        instructions = QtGui.QLabel("Draw a diameter in the image below across the pupil, then press Done")
        done_btn = QtGui.QPushButton("Done")
        image_panel = QtGui.QLabel()
        image_panel.setPixmap(pixmap)

        grid.addWidget(instructions,0,0)
        grid.addWidget(done_btn,1,5)
        grid.addWidget(image_panel,2,0,1,6)

        done_btn.clicked.connect(self.done)

        self.click_pos = None
        self.release_pos = None

        self.setWindowTitle("Set Radius Estimate")
        self.move(50,50)
        self.show()

    def mousePressEvent(self,QMouseEvent):
        self.click_pos = (QMouseEvent.x(),QMouseEvent.y())
        print self.click_pos

    def mouseReleaseEvent(self,QMouseEvent):
        self.release_pos = (QMouseEvent.x(),QMouseEvent.y())
        print self.release_pos

    def done(self):
        expected_pupil_radius = int(math.sqrt((self.click_pos[0] - self.release_pos[0])**2 + (self.click_pos[1] - self.release_pos[1])**2)/2)
        self.CMOSthread.expected_pupil_radius = expected_pupil_radius
        self.CMOSthread.app.radius_entry.setText(str(expected_pupil_radius))
        self.close()


class CMOSthread(QtCore.QThread):

    def __init__(self,app):
        super(CMOSthread,self).__init__()

        self.app = app
        self.mako = app.mako
        self.stop_event = app.stop_event

        self.frame = self.mako.camera.getFrame()
        self.frame.announceFrame()
        self.qImage = None

        self.pupil_video_frames = []
        self.pupil_data_list = []
        self.medianBlur = None
        self.dp = None
        self.minDist = None
        self.param1 = None
        self.param2 = None
        self.radius_range = None
        self.expected_pupil_radius = None
        self.popup = None

    def apply_parameters(self):
        self.medianBlur = int(self.app.blur_entry.displayText())
        self.dp = float(self.app.dp_entry.displayText())
        self.minDist = int(self.app.minDist_entry.displayText())
        self.param1 = int(self.app.param1_entry.displayText())
        self.param2 = int(self.app.param2_entry.displayText())
        self.radius_range = int(self.app.range_entry.displayText())
        self.expected_pupil_radius = int(self.app.radius_entry.displayText())

    def ask_radius_estimate(self):
        self.popup = Popup(self)

    def trigger_record(self):
        if not self.app.record_btn.isChecked():
            written_video_frames = 0
            pupil_video_writer = skv.FFmpegWriter('data_acquisition/pupil_video.avi',outputdict={
            '-vcodec':'libx264',
            '-b':'30000000',
            '-vf':'setpts=4*PTS',
            '-r':'20'})

            for frame in self.pupil_video_frames:
                pupil_video_writer.writeFrame(frame)
                written_video_frames += 1
                print "wrote a frame!"
            
            print "finished writing!"
            pupil_video_writer.close()
            self.pupil_video_frames = []

            frame_number = 0
            pupil_data_file = open('data_acquisition/pupil_data.txt','w+')

            for frame_number in range(1,len(self.pupil_data_list)+1):
                pupil_center, pupil_radius = self.pupil_data_list[frame_number-1]
                if pupil_center is None or pupil_radius is None: 
                    pupil_data_file.write("Frame: %d No pupil detected!\n" % frame_number)
                else: 
                    pupil_data_file.write("Frame: %d Pupil Center: %s Pupil Radius: %d\n" % (frame_number,pupil_center,pupil_radius))

            print "video frames vs data frames",written_video_frames,frame_number
            pupil_data_file.close()
            self.pupil_data_list = []

    def run(self):
        self.mako.camera.startCapture()
        self.mako.camera.runFeatureCommand('AcquisitionStart')
        self.frame.queueFrameCapture()

        while not self.stop_event.is_set():
            #print "videoLoop"
            # self.root.update()
            self.frame.waitFrameCapture(1000)
            self.frame.queueFrameCapture()
            imgData = self.frame.getBufferByteData()
            image_arr = np.ndarray(buffer = imgData,
                           dtype = np.uint8,
                           shape = (self.frame.height,self.frame.width))    
            
            
            resized_image = imutils.resize(image_arr, width=1024)
            plain_image = cv2.cvtColor(resized_image, cv2.COLOR_GRAY2BGR)

            if self.medianBlur is None or self.dp is None or self.minDist is None or self.param1 is None or self.param2 is None or self.radius_range is None or self.expected_pupil_radius is None:
                pupil_data = [plain_image,None,None]
            else:
                pupil_data = ht.detect_pupil_frame(plain_image,self.medianBlur,self.dp,self.minDist,self.param1,self.param2,self.radius_range,self.expected_pupil_radius)


            if self.app.record_btn.isChecked(): # extra check to cover for out incorrect ordering case

                self.pupil_video_frames.append(pupil_data[0].copy())
                self.pupil_data_list.append((pupil_data[1],pupil_data[2]))
                print "added frame to list"
            

            # convets image to form used by tkinter
            image = cv2.cvtColor(pupil_data[0].copy(), cv2.COLOR_BGR2RGB)
            height, width, channel = image.shape
            bytesPerLine = 3 * width
            qImage = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 

            self.qImage = qImage
            
            self.emit(QtCore.SIGNAL('update_CMOS_panel(PyQt_PyObject)'),self.qImage)

        
class EMCCDthread(QtCore.QThread):

    camera_signal = QtCore.pyqtSignal('PyQt_PyObject')
    graph_signal = QtCore.pyqtSignal('PyQt_PyObject')
    curve_signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self,camera,motor,graph):
        super(EMCCDthread,self).__init__()

        self.andor = camera
        self.motor = motor
        self.graph = graph
        self.FSR_entry = None
        self.SD_entry = None
        self.stop_event = App.stop_event
        self.condition = App.condition

        self.image = None
        self.image_andor = None
        self.analyzed_row = np.zeros(80)

        self.export_list = []
        self.brillouin_shift_list = []
        self.shutter_state = False
        self.scan_ready = False

    
    def scan(self, start_pos, length, num_steps):
        self.motor.device.move_abs(int(start_pos/3.072))
        step_size = length // num_steps

        self.export_list.append(self.acquire_frame()) #initial frame

        for i in range(num_steps):

            self.motor.device.move_rel(int(step_size/3.072))           
            
            self.export_list.append(self.acquire_frame())

        if len(self.export_list) != 0:
            self.export_list[0].save("data_acquisition/scan.tif",compression="tiff_deflate",save_all=True,append_images=self.export_list[1:]) 
            self.export_list = []
            print "finished exporting as tif"

    def set_shutter_state(self,state):
        self.shutter_state = state

    def set_entries(self,FSR_entry,SD_entry):
        self.FSR_entry = FSR_entry
        self.SD_entry = SD_entry

    def acquire_frame(self):
        self.andor.cam.StartAcquisition() 
        data = []                                            
        self.andor.cam.GetAcquiredData(data)

        image_array = np.array(data, dtype = np.uint16)
        maximum = image_array.max()

        proper_image = np.reshape(image_array, (-1, 512))

        scaled_image = proper_image*(255.0/maximum)
        scaled_image = scaled_image.astype(int)

        return np.array(scaled_image, dtype = np.uint8)

    def run(self):
        while not self.stop_event.is_set():
            #print "andorLoop"
            scaled_8bit = self.acquire_frame()   

            loc = np.argmax(scaled_8bit)/512
            left_right = scaled_8bit[loc].argsort()[-10:][::-1]
            left_right.sort()
            mid = int((left_right[0]+left_right[-1])/2)

            ################
            ### GRAPHING ###
            ################

            graph_data = list(data)
            graph_array = np.array(graph_data, dtype = np.uint16)
            reshaped_graph = np.reshape(graph_array, (-1,512))

            self.analyzed_row = reshaped_graph[loc][mid-40:mid+40]

            self.graphLoop()

            ###############

            cropped = scaled_8bit[loc-7:loc+7, mid-40:mid+40]

            (h, w)= cropped.shape[:2]
            if w <= 0 or h <= 0:
                continue

            self.image_andor = cropped

            image = imutils.resize(self.image_andor, width=1024)
            bgr_image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)

            height, width, channel = bgr_image.shape
            bytesPerLine = 3 * width
            qImage = QtGui.QImage(bgr_image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 

            self.camera_signal.emit(qImage)


    #plots graphs, similar to how graphs are plotted on andoru_test.py
    #uses figure so both plots can be shown at same time
    def graphLoop(self):

        copied_analyzed_row = np.array(self.analyzed_row)

        try:
            FSR = float(self.FSR_entry.displayText())
            SD = float(self.SD_entry.displayText())
            
            if not self.shutter_state:
                constant_1 = np.amax(copied_analyzed_row[:40])
                constant_2 = np.amax(copied_analyzed_row[40:])
                x0_1 = np.argmax(copied_analyzed_row[:40])
                x0_2 = np.argmax(copied_analyzed_row[40:])+40
                for i in xrange(40 - x0_1): 
                    half_max = constant_1/2
                    if copied_analyzed_row[:40][x0_1+i] <= half_max:
                        gamma_1 = i*2
                        break
                for j in xrange(40 - (x0_2 - 40)):
                    half_max = constant_2/2
                    if copied_analyzed_row[40:][x0_2 - 40+j] <= half_max:
                        gamma_2 = j*2
                        break

                delta_peaks = x0_2 - x0_1
                BS = (FSR - delta_peaks*SD)/2
                length_bs = len(self.brillouin_shift_list)
                if length_bs >= 100:
                    self.brillouin_shift_list = []
                self.brillouin_shift_list.append(BS)
                length_bs +=1

                
                
                popt, pcov = curve_fit(lorentzian, self.graph.x_axis, copied_analyzed_row, p0 = np.array([gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, 100]))
                #subplot.plot(self.graph.x_axis, lorentzian(self.graph.x_axis, *popt), 'r-', label='fit')
               
                curve_data = (popt,pcov)
                self.curve_signal.emit(curve_data)
            else:
                constant_1 = np.amax(copied_analyzed_row[:20])
                constant_2 = np.amax(copied_analyzed_row[20:40])
                constant_3 = np.amax(copied_analyzed_row[40:60])
                constant_4 = np.amax(copied_analyzed_row[60:])
                constant_5 = 100


                x0_1 = np.argmax(copied_analyzed_row[:20])
                x0_2 = np.argmax(copied_analyzed_row[20:40])+20
                x0_3 = np.argmax(copied_analyzed_row[40:60])+40
                x0_4 = np.argmax(copied_analyzed_row[60:])+60



                popt, pcov = curve_fit(lorentzian_reference, self.graph.x_axis, copied_analyzed_row, p0 = np.array([1, x0_1, constant_1, 1, x0_2, constant_2, 1, x0_3, constant_3, 1, x0_4, constant_4, constant_5]))
                #subplot.plot(self.graph.x_axis, lorentzian_reference(self.graph.x_axis, *popt), 'r-', label='fit')
                measured_SD = (2*self.PlasticBS - 2*self.WaterBS) / ((x0_4 - x0_1) + (x0_3 - x0_2))
                measured_FSR = 2*self.PlasticBS - measured_SD*(x0_3 - x0_2)
                
                curve_data = (popt,pcov,measured_SD,measured_FSR)
                self.curve_signal.emit(curve_data)

                #self.SD.set(measured_SD)
                #self.FSR.set(measured_FSR)

        except Exception as e:
            traceback.print_exc()
            pass

        graph_data = (copied_analyzed_row.copy(),self.brillouin_shift_list[:])
        self.graph_signal.emit(graph_data)


class Graph(object):
    def __init__(self):
        self.fig = Figure(figsize = (10, 10), dpi = 100)
        self.x_axis = np.arange(1,81)

def lorentzian(x, gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, constant_3):
    numerator_1 = 0.5*gamma_1*constant_1
    denominator_1 = math.pi * (x - x0_1) **2 + (0.5 * gamma_1)**2
    numerator_2 = 0.5*gamma_2*constant_2
    denominator_2 = math.pi * (x - x0_2) **2 + (0.5 * gamma_2)**2
    y = (numerator_1/denominator_1) + (numerator_2/denominator_2) + constant_3
    return y

def lorentzian_reference(x, gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, gamma_3, x0_3, constant_3, gamma_4, x0_4, constant_4, constant_5):
    numerator_1 = 0.5*gamma_1*constant_1
    denominator_1 = math.pi * (x - x0_1) **2 + (0.5 * gamma_1)**2
    numerator_2 = 0.5*gamma_2*constant_2
    denominator_2 = math.pi * (x - x0_2) **2 + (0.5 * gamma_2)**2
    numerator_3 = 0.5*gamma_3*constant_3
    denominator_3 = math.pi * (x - x0_3) **2 + (0.5 * gamma_3)**2
    numerator_4 = 0.5*gamma_4*constant_4
    denominator_4 = math.pi * (x - x0_4) **2 + (0.5 * gamma_4)**2
    y = (numerator_1/denominator_1) + (numerator_2/denominator_2) + (numerator_3/denominator_3) + (numerator_4/denominator_4) + constant_5
    return y
        






if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    GUI = App()
    sys.exit(app.exec_())

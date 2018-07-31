
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

import CMOSthread
import EMCCDthread

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
        self.andor_lock = threading.Lock()
        self.condition = threading.Condition()

        # initialize and access cameras, motors and graphs
        self.mako = device_init.Mako_Camera()
        self.andor = device_init.Andor_Camera()
        self.motor = device_init.Motor()
        self.graph = EMCCDthread.Graph()

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
        self.coord_panel_image = np.zeros((1030,1030,3), np.uint8)

        self.CMOSthread = CMOSthread.CMOSthread(self)
        self.connect(self.CMOSthread,QtCore.SIGNAL('update_CMOS_panel(PyQt_PyObject)'),self.update_CMOS_panel)


        self.EMCCDthread = EMCCDthread.EMCCDthread(self)
        self.connect(self.EMCCDthread,QtCore.SIGNAL('update_EMCCD_panel(PyQt_PyObject)'),self.update_EMCCD_panel)
        #self.EMCCDthread.graph_signal.connect(self.update_graph_panel)
        #self.EMCCDthread.curve_signal.connect(self.draw_curve)

        ### CMOS AND EMCCD PANEL ###
        self.cmos_panel = QtGui.QLabel()
        self.cmos_panel.mousePressEvent = self.handle_click
        self.emccd_panel = QtGui.QLabel()
        self.canvas = FigureCanvasQTAgg(self.graph.fig)

        self.mainUI()

        self.CMOSthread.start()
        self.EMCCDthread.start()

    def mainUI(self):
        grid = QtGui.QGridLayout()
        self.setLayout(grid)


        grid.addWidget(self.cmos_panel,0,4,11,7)
        grid.addWidget(self.emccd_panel,0,11,6,5)
        grid.addWidget(self.canvas,6,11,3,5)

        self.cmos_panel.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
        
        #############################
        ### PUPIL DETECTION PANEL ###
        #############################

        det_grid = QtGui.QGridLayout()
        grid.addLayout(det_grid,0,0,11,4)

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
        set_coordinates_btn = QtGui.QPushButton("Set Coordinates",self)
        set_scan_loc_btn = QtGui.QPushButton("Set Scan Location",self)
        set_scan_loc_btn.setCheckable(True)
        coord_panel = QtGui.QLabel()
        scanned_loc_table = QtGui.QTableWidget(1,4,self)

        resized_image = imutils.resize(self.coord_panel_image.copy(), width=self.coord_panel_image.shape[0]/2)
        coord_pixmap = self.convert_to_pixmap(resized_image)
        coord_panel.setPixmap(coord_pixmap)
        coord_panel.show()

        headers = [QtCore.QString("pos (pixels)"),QtCore.QString("start pos (um)"),QtCore.QString("scan length (um)"),QtCore.QString("#frames")]
        horizontal_headers = QtCore.QStringList()
        for header in headers:
            horizontal_headers.append(header)
        scanned_loc_table.setHorizontalHeaderLabels(horizontal_headers)
        scanned_loc_table.resizeColumnsToContents()
        scanned_loc_table.horizontalHeader().setStretchLastSection(True)

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
        det_grid.addWidget(set_coordinates_btn, 12, 0, 1, 2)
        det_grid.addWidget(set_scan_loc_btn,13, 0, 1, 2)
        det_grid.addWidget(coord_panel,0, 2, 4, 2)
        det_grid.addWidget(scanned_loc_table, 4, 2, 8, 2)

        detection_panel_label.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        blur_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        dp_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        minDist_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        param1_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        param2_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        range_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        radius_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        coord_panel.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignTop)

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
        self.set_scan_loc_btn = set_scan_loc_btn
        self.coord_panel = coord_panel
        self.scanned_loc_table = scanned_loc_table

        radius_btn.clicked.connect(self.CMOSthread.ask_radius_estimate)
        default_btn.clicked.connect(self.restore_default_params)
        apply_btn.clicked.connect(self.CMOSthread.apply_parameters)
        set_coordinates_btn.clicked.connect(self.CMOSthread.set_coordinates)


        ##########################
        ### PUPIL CAMERA PANEL ###
        ##########################

        cam_grid = QtGui.QGridLayout()
        grid.addLayout(cam_grid,11,4,1,2)

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
        grid.addLayout(graph_grid,11,6,1,5)

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
        grid.addLayout(motor_grid,12,4,2,7)

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
        forward_btn.clicked.connect(lambda: self.move_motor_rel(float(self.distance_entry.displayText())))
        backward_btn.clicked.connect(lambda: self.move_motor_rel(-float(self.distance_entry.displayText())))
        position_btn.clicked.connect(lambda: self.move_motor_abs(float(self.location_entry.displayText())))

        #############################
        ### DATA COLLECTION PANEL ###
        #############################

        data_grid = QtGui.QGridLayout()
        grid.addLayout(data_grid,14,4,2,4)

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

        scan_btn.clicked.connect(lambda: self.EMCCDthread.scan(float(start_pos.displayText()),float(scan_length.displayText()),int(num_frames.displayText())))

        #######################################
        ### VELOCITY AND ACCELERATION PANEL ###
        #######################################

        vel_grid = QtGui.QGridLayout()
        grid.addLayout(vel_grid,14,8,2,2)

        velocity_label = QtGui.QLabel("Velocity")
        velocity_entry = QtGui.QLineEdit()
        velocity_btn = QtGui.QPushButton("Change Velocity",self)

        vel_grid.addWidget(velocity_label, 0, 0)
        vel_grid.addWidget(velocity_entry, 1, 0)
        vel_grid.addWidget(velocity_btn, 1, 1)


        self.setWindowTitle("Brillouin Scan Interface")
        self.move(50,50)
        self.show()
        print "finished showing"

    def update_CMOS_panel(self,qImage):
        #print "updating CMOS panel"
        #updating cmos panel
        pixmap = QtGui.QPixmap.fromImage(qImage)
        self.cmos_panel.setPixmap(pixmap)
        self.cmos_panel.show()

        #update coord panel
        if self.CMOSthread.detected_radius is not None and self.CMOSthread.detected_center is not None:
            circle_image = self.coord_panel_image.copy()
            image_dim = circle_image.shape
            image_center = (image_dim[1]/2,image_dim[0]/2)

            if self.CMOSthread.scan_loc is not None:
                size = 20
                scan_loc = self.CMOSthread.scan_loc
                det_center = self.CMOSthread.detected_center
                rel_scan_loc = (scan_loc[0]-det_center[0],scan_loc[1]-det_center[1])
                panel_loc = (rel_scan_loc[0]+image_center[0],rel_scan_loc[1]+image_center[1])

                cv2.line(circle_image,(panel_loc[0],min(panel_loc[1]+size,image_dim[0])),(panel_loc[0],max(panel_loc[1]-size,0)),(0,255,0),1)
                cv2.line(circle_image,(min(panel_loc[0]+size,image_dim[1]),panel_loc[1]),(max(panel_loc[0]-size,0),panel_loc[1]),(0,255,0),1)
            
            cv2.circle(circle_image,image_center,self.CMOSthread.detected_radius,(255,0,0),2)

            resized_image = imutils.resize(circle_image, width=image_dim[0]/2)

            image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            height, width, channel = image.shape
            bytesPerLine = 3 * width
            coord_image = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 
            pixmap = QtGui.QPixmap.fromImage(coord_image)
            self.coord_panel.setPixmap(pixmap)
            self.coord_panel.show()


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

    def handle_click(self,event):
        
        if self.set_scan_loc_btn.isChecked():
            x = event.pos().x()
            y = event.pos().y()
            self.CMOSthread.scan_loc = (x,y)
            self.set_scan_loc_btn.setChecked(False)
            print (x,y)


    def convert_to_pixmap(self,image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channel = image.shape
        bytesPerLine = 3 * width
        qimage = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 
        return QtGui.QPixmap.fromImage(qimage)


    #similar to shutters.py, called on by reference button 
    def shutters(self, close = False):
        dll = WinDLL("C:\\Program Files\\quad-shutter\\Quad Shutter dll and docs\\x64\\PiUsb")
        c2 = c_int()
        c4 = c_int()
        usb312 = dll.piConnectShutter(byref(c2), 312)
        usb314 = dll.piConnectShutter(byref(c4), 314)

        
        state = self.reference_btn.isChecked()
            
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
        self.location_entry.setText(str(loc.data*3.072))

    # moves zaber motor, called on by forwards and backwards buttons
    def move_motor_rel(self,distance):
        self.motor.device.move_rel(distance/3.072)
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(loc.data*3.072))

     # moves zaber motor to a set location, called on above
    def move_motor_abs(self,pos):
        self.motor.device.move_abs(pos/3.072)
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(loc.data*3.072))

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
        

        






if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    GUI = App()
    sys.exit(app.exec_())

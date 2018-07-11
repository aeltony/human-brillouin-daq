
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
import Queue
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
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.optimize import curve_fit

class App(QtGui.QWidget):

    def __init__(self):
        super(App,self).__init__()

        self.stopEvent = threading.Event()


        # initialize and access cameras, motors and graphs
        self.mako = device_init.Mako_Camera()
        #self.andor = device_init.Andor_Camera()
        #self.motor = device_init.Motor()
        #self.graph = Graph()

        self.CMOSthread = CMOSthread(self.mako,self.stopEvent)
        self.CMOSthread.signal.connect(self.update_CMOS_panel)
        ### CMOS PANEL ###
        self.cmos_panel = QtGui.QLabel()

        self.mainUI()
        self.CMOSthread.start()

    def mainUI(self):


        grid = QtGui.QGridLayout()
        self.setLayout(grid)

        grid.addWidget(self.cmos_panel,0,0,3,6)


        ### PUPIL CAMERA PANEL ###
        snapshot_btn = QtGui.QPushButton("Take Picture",self)
        record_btn = QtGui.QPushButton("Record",self)
        record_btn.setCheckable(True)

        grid.addWidget(snapshot_btn,3,0)
        grid.addWidget(record_btn,3,1)

        ### GRAPH PANEL ###
        reference_btn = QtGui.QPushButton("Reference",self)
        reference_btn.setCheckable(True)
        FSR_label = QtGui.QLabel("FSR")
        FSR_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        FSR_entry = QtGui.QLineEdit()
        SD_label = QtGui.QLabel("SD")
        SD_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        SD_entry = QtGui.QLineEdit()

        grid.addWidget(reference_btn,3,2)
        grid.addWidget(FSR_label,3,3)
        grid.addWidget(FSR_entry,3,4)
        grid.addWidget(SD_label,3,5)
        grid.addWidget(SD_entry,3,6)

        ### MOTOR PANEL ###
        motor_label = QtGui.QLabel("Motor Control")
        home_btn = QtGui.QPushButton("Home",self)
        distance_label = QtGui.QLabel("Distance To Move")
        distance_entry = QtGui.QLineEdit()
        forward_btn = QtGui.QPushButton("Forward",self)
        backward_btn = QtGui.QPushButton("Backward",self)
        location_label = QtGui.QLabel("Location")
        location_entry = QtGui.QLineEdit()
        position_btn = QtGui.QPushButton("Move To Location",self)

        grid.addWidget(motor_label,4,0)
        grid.addWidget(home_btn,5,0)
        grid.addWidget(distance_label,4,1)
        grid.addWidget(distance_entry,5,1)
        grid.addWidget(forward_btn,5,2)
        grid.addWidget(backward_btn,5,3)
        grid.addWidget(location_label,4,5)
        grid.addWidget(location_entry,5,5)
        grid.addWidget(position_btn,5,6)

        ### DATA COLLECTION PANEL ###
        start_pos_label = QtGui.QLabel("Start Position(um)")
        start_pos = QtGui.QLineEdit()
        num_frames_label = QtGui.QLabel("#Frames")
        num_frames = QtGui.QLineEdit()
        scan_length_label = QtGui.QLabel("Length(um)")
        scan_length = QtGui.QLineEdit()
        scan_btn = QtGui.QPushButton("Start Scan",self)

        grid.addWidget(start_pos_label,6,0)
        grid.addWidget(start_pos,7,0)
        grid.addWidget(num_frames_label,6,1)
        grid.addWidget(num_frames,7,1)
        grid.addWidget(scan_length_label,6,2)
        grid.addWidget(scan_length,7,2)
        grid.addWidget(scan_btn,7,3)

        ### VELOCITY AND ACCELERATION PANEL ###
        velocity_label = QtGui.QLabel("Velocity")
        velocity = QtGui.QLineEdit()
        velocity_btn = QtGui.QPushButton("Change Velocity",self)

        grid.addWidget(velocity_label,6,4)
        grid.addWidget(velocity,7,4)
        grid.addWidget(velocity_btn,7,5)


        self.setWindowTitle("Brillouin Scan Interface")
        self.move(50,50)
        #self.resize(800,800)
        self.show()
        print "finished showing"

    def update_CMOS_panel(self,qImage):
        print "updating CMOS panel"
        pixmap = QtGui.QPixmap.fromImage(qImage)
        self.cmos_panel.setPixmap(pixmap)
        self.cmos_panel.show()

    def closeEvent(self,event):
        self.stopEvent.set()
        self.mako.camera.runFeatureCommand('AcquisitionStop')
        self.mako.camera.endCapture()
        self.mako.camera.revokeAllFrames()
        self.mako.vimba.shutdown()

        event.accept()


class CMOSthread(QtCore.QThread):

    signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self,camera,stopEvent):
        super(CMOSthread,self).__init__()

        self.mako = camera
        self.stopEvent = stopEvent
        self.frame = self.mako.camera.getFrame()
        self.frame.announceFrame()
        self.image = None

    def run(self):
        self.mako.camera.startCapture()
        self.mako.camera.runFeatureCommand('AcquisitionStart')
        self.frame.queueFrameCapture()

        while not self.stopEvent.is_set():
            print "videoLoop"
            # self.root.update()
            self.frame.waitFrameCapture(1000)
            self.frame.queueFrameCapture()
            imgData = self.frame.getBufferByteData()
            image = np.ndarray(buffer = imgData,
                           dtype = np.uint8,
                           shape = (self.frame.height,self.frame.width))    
            
            
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            """
            if self.click_pos is not None and self.release_pos is not None:
                expected_pupil_radius = int(math.sqrt((self.click_pos[0] - self.release_pos[0])**2 + (self.click_pos[1] - self.release_pos[1])**2)/2)
                pupil_data = ht.detect_pupil_frame(image,expected_pupil_radius,15)
            else:
                pupil_data = ht.detect_pupil_frame(image)


            if self.record.get() == 1:
                with self.record_lock:

                    self.pupil_video_frames.append(pupil_data[0].copy())
                    self.pupil_data_list.append((pupil_data[1],pupil_data[2]))
                    print "added frame to list"
            """     

            self.image = image
            # convets image to form used by tkinter
            image = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2RGB)
            image = imutils.resize(image, width=1024)
            height, width, channel = image.shape
            bytesPerLine = 3 * width
            qImage = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 

            self.signal.emit(qImage)
            #image = Image.fromarray(image)
            #image = ImageTk.PhotoImage(image)
        




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

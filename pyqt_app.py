
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
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from scipy.optimize import curve_fit

class App(QtGui.QWidget):

    #Lock used to halt other threads upon app closing
    stop_event = threading.Event()
    condition = threading.Condition()

    def __init__(self):
        super(App,self).__init__()

        # initialize and access cameras, motors and graphs
        self.mako = device_init.Mako_Camera()
        self.andor = device_init.Andor_Camera()
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
        self.click_pos = None
        self.release_pos = None
        self.expected_pupil_radius = 0

        self.CMOSthread = CMOSthread(self.mako)
        self.CMOSthread.camera_signal.connect(self.update_CMOS_panel)

        self.EMCCDthread = EMCCDthread(self.andor,self.graph)
        self.EMCCDthread.camera_signal.connect(self.update_EMCCD_panel)
        self.EMCCDthread.graph_signal.connect(self.update_graph_panel)
        #self.EMCCDthread.curve_signal.connect(self.draw_curve)

        ### CMOS AND EMCCD PANEL ###
        self.cmos_panel = QtGui.QLabel()
        self.emccd_panel = QtGui.QLabel()
        self.canvas = FigureCanvasQTAgg(self.graph.fig)

        self.mainUI()
        self.EMCCDthread.set_entries(self.FSR_entry,self.SD_entry)

        self.CMOSthread.start()
        self.EMCCDthread.start()

    def mainUI(self):
        grid = QtGui.QGridLayout()
        self.setLayout(grid)

        grid.addWidget(self.cmos_panel,0,0,3,7)
        grid.addWidget(self.emccd_panel,0,7,1,6)
        grid.addWidget(self.canvas,1,7,3,6)
        
        ##########################
        ### PUPIL CAMERA PANEL ###
        ##########################

        snapshot_btn = QtGui.QPushButton("Take Picture",self)
        record_btn = QtGui.QPushButton("Record",self)
        record_btn.setCheckable(True)

        grid.addWidget(snapshot_btn,3,0)
        grid.addWidget(record_btn,3,1)

        record_btn.clicked.connect(self.CMOSthread.trigger_record)

        self.record_btn = record_btn
        
        ###################
        ### GRAPH PANEL ###
        ###################

        reference_btn = QtGui.QPushButton("Reference",self)
        reference_btn.setCheckable(True)
        FSR_label = QtGui.QLabel("FSR")
        FSR_entry = QtGui.QLineEdit()
        SD_label = QtGui.QLabel("SD")
        SD_entry = QtGui.QLineEdit()

        grid.addWidget(reference_btn,3,2)
        grid.addWidget(FSR_label,3,3)
        grid.addWidget(FSR_entry,3,4)
        grid.addWidget(SD_label,3,5)
        grid.addWidget(SD_entry,3,6)

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

        #######################################
        ### VELOCITY AND ACCELERATION PANEL ###
        #######################################

        velocity_label = QtGui.QLabel("Velocity")
        velocity = QtGui.QLineEdit()
        velocity_btn = QtGui.QPushButton("Change Velocity",self)

        grid.addWidget(velocity_label,6,4)
        grid.addWidget(velocity,7,4)
        grid.addWidget(velocity_btn,7,5)


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
        self.motor.device.move_rel(int(distance/3.072))
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(int(loc.data*3.072)))

     # moves zaber motor to a set location, called on above
    def move_motor_abs(self,pos):
        self.motor.device.move_abs(int(pos/3.072))
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(int(loc.data*3.072)))

    def set_velocity(self, velocity):
        self.motor.device.send(42,velocity)

    def slice_routine(self, start_pos, length, num_steps):
        self.move_motor_abs(start_pos)
        step_size = length // num_steps

        for i in range(num_steps):
            self.condition.acquire()
            #print "slice_routine acquired lock"

            self.move_motor_relative(step_size)            
            #print "moving.. "
            
            self.EMCCDthread.trigger_scan()

            #print "waiting.."
            self.condition.wait()

        self.EMCCDthread.export_images()


    def closeEvent(self,event):
        self.stop_event.set()
        self.mako.camera.runFeatureCommand('AcquisitionStop')
        self.mako.camera.endCapture()
        self.mako.camera.revokeAllFrames()
        self.mako.vimba.shutdown()
        self.motor.port.close()
        self.shutters(close = True)

        event.accept() #closes the application


class CMOSthread(QtCore.QThread):

    camera_signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self,camera):
        super(CMOSthread,self).__init__()

        self.mako = camera
        self.stop_event = App.stop_event

        self.frame = self.mako.camera.getFrame()
        self.frame.announceFrame()
        self.image = None

        self.record = False
        self.pupil_video_frames = []
        self.pupil_data_list = []

    def trigger_record(self):

        self.record = not self.record

        if not self.record:
            written_video_frames = 0
            pupil_video_writer = skv.FFmpegWriter('data_acquisition/pupil_video.avi',outputdict={
            '-vcodec':'libx264',
            '-b':'30000000',
            '-vf':'setpts=4*PTS'
            })

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
            """
            pupil_data = ht.detect_pupil_frame(image)

            if self.record: # extra check to cover for out incorrect ordering case
                self.pupil_video_frames.append(pupil_data[0].copy())
                self.pupil_data_list.append((pupil_data[1],pupil_data[2]))
                print "added frame to list"
            

            self.image = image
            # convets image to form used by tkinter
            image = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2RGB)
            image = imutils.resize(image, width=1024)
            height, width, channel = image.shape
            bytesPerLine = 3 * width
            qImage = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 

            self.camera_signal.emit(qImage)
            #image = Image.fromarray(image)
            #image = ImageTk.PhotoImage(image)
        
class EMCCDthread(QtCore.QThread):

    camera_signal = QtCore.pyqtSignal('PyQt_PyObject')
    graph_signal = QtCore.pyqtSignal('PyQt_PyObject')
    curve_signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self,camera,graph):
        super(EMCCDthread,self).__init__()

        self.andor = camera
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

    def trigger_scan(self):
        self.scan_ready = True

    def export_images(self):
        if len(self.export_list) != 0:
            self.export_list[0].save("data_acquisition/scan.tif",compression="tiff_deflate",save_all=True,append_images=self.export_list[1:]) 
            self.export_list = []
            print "finished exporting as tif"


    def set_shutter_state(self,state):
        self.shutter_state = state

    def set_entries(self,FSR_entry,SD_entry):
        self.FSR_entry = FSR_entry
        self.SD_entry = SD_entry

    def run(self):
        while not self.stop_event.is_set():
            #print "andorLoop"
            self.andor.cam.StartAcquisition() 
            data = []                                            
            self.andor.cam.GetAcquiredData(data)

            image_array = np.array(data, dtype = np.uint16)
            maximum = image_array.max()


            graph_data = list(data)  
            graph_array = np.array(graph_data, dtype = np.uint16)
            reshaped_graph = np.reshape(graph_array, (-1, 512))

            proper_image = np.reshape(image_array, (-1, 512))


            scaled_image = proper_image*(255.0/maximum)
            scaled_image = scaled_image.astype(int)
            scaled_8bit= np.array(scaled_image, dtype = np.uint8)

            
            if self.scan_ready: 
                self.condition.acquire()

                self.export_list.append(Image.fromarray(scaled_8bit))

                self.scan_ready = False

                self.condition.notifyAll()
                self.condition.release()
            

            loc = np.argmax(scaled_8bit)/512
            left_right = scaled_8bit[loc].argsort()[-10:][::-1]
            left_right.sort()
            mid = int((left_right[0]+left_right[-1])/2)

        
            self.analyzed_row = reshaped_graph[loc][mid-40:mid+40]


            cropped = scaled_8bit[loc-7:loc+7, mid-40:mid+40]
            self.graphLoop()

            (h, w)= cropped.shape[:2]
            if w <= 0 or h <= 0:
                continue

            self.image_andor = cropped

            image = imutils.resize(self.image_andor, width=1024)
            bgr_image = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
            print bgr_image.shape
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

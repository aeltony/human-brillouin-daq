import threading
import datetime
import imutils
import cv2
import numpy as np
import time
import math
import sys
import os
import traceback

from PyQt4 import QtGui,QtCore
from PIL import Image
from PIL import ImageTk
import hough_transform as ht
import skvideo.io as skv

import CMOSthread
import EMCCDthread

# UI import
import qt_ui

# device imports

import device_init
from pymba import *
from my_andor.andor_wrap import *
from ctypes import *
import zaber.serial as zs

# graphing imports
import matplotlib
matplotlib.use('Qt4Agg')
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from scipy.optimize import curve_fit
from scipy.interpolate import griddata
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

class App(QtGui.QMainWindow,qt_ui.Ui_MainWindow):
 
    def __init__(self):
        super(App,self).__init__()
        self.setupUi(self)

        self.output_path = None

        #Lock used to halt other threads upon app closing
        self.stop_event = threading.Event()
        self.andor_lock = threading.Lock()
        self.scan_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.condition = threading.Condition()

        # initialize and access cameras, motors and graphs
        self.mako = device_init.Mako_Camera()
        self.andor = device_init.Andor_Camera()
        self.motor = device_init.Motor()
        self.graph = EMCCDthread.Graph()
        self.avg_heatmap = EMCCDthread.HeatMapGraph(25,-1)

        self.brillouin_shift_list = []
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
        
        self.detected_center = None
        self.detected_radius = None
        self.scanned_locations = {} #maps unique id to scanned location in coordinate system
        self.current_ID = 10000

        self.heatmaps = {-1:self.avg_heatmap}

        self.coord_panel_image = None

        self.CMOSthread = CMOSthread.CMOSthread(self)
        self.connect(self.CMOSthread,QtCore.SIGNAL('update_CMOS_panel(PyQt_PyObject)'),self.update_CMOS_panel)

        self.EMCCDthread = EMCCDthread.EMCCDthread(self)
        self.connect(self.EMCCDthread,QtCore.SIGNAL('update_EMCCD_panel(PyQt_PyObject)'),self.update_EMCCD_panel)
        self.connect(self.EMCCDthread,QtCore.SIGNAL('update_graph_panel(PyQt_PyObject)'),self.update_graph_panel)
        self.connect(self.EMCCDthread,QtCore.SIGNAL('draw_curve(PyQt_PyObject)'),self.draw_curve)
        self.connect(self.EMCCDthread,QtCore.SIGNAL('update_heatmap_panel(PyQt_PyObject)'),self.update_heatmap_panel)

        ### CMOS AND EMCCD PANEL ###
        self.cmos_panel.mousePressEvent = self.handle_click
        self.graph_panel.initialize_canvas(self.graph.fig)

        self.avg_heatmap.plot()
        print self.avg_heatmap.ax
        self.heatmap_panel.initialize_canvas(self.avg_heatmap.fig)


        self.heatmap_panel.canvas.draw()

        self.mainUI()

        self.CMOSthread.start()
        self.EMCCDthread.start()

    def mainUI(self):

        self.screenshot_btn.clicked.connect(self.take_screenshot)

        self.scan_images.setIconSize(QtCore.QSize(1024,1024))

        #############################
        ### PUPIL DETECTION PANEL ###
        #############################

        self.radius_btn.clicked.connect(self.CMOSthread.ask_radius_estimate)
        self.default_btn.clicked.connect(self.restore_default_params)
        self.apply_btn.clicked.connect(self.CMOSthread.apply_parameters)
        self.set_coordinates_btn.clicked.connect(self.CMOSthread.set_coordinates)

        #################################
        ### COORDINATE TRACKING PANEL ###
        #################################

        headers = [QtCore.QString("pos (pixels)"),QtCore.QString("average shift (GHz)"),QtCore.QString("start pos (um)"),QtCore.QString("scan length (um)"),QtCore.QString("#frames"),QtCore.QString("ID")]
        horizontal_headers = QtCore.QStringList()
        for header in headers:
            horizontal_headers.append(header)
        self.scanned_loc_table.setHorizontalHeaderLabels(horizontal_headers)
        self.scanned_loc_table.resizeColumnsToContents()
        self.scanned_loc_table.horizontalHeader().setStretchLastSection(True)

        #self.slider.valueChanged.connect(self.change_heatmap_depth)

        self.delete_btn.clicked.connect(self.delete_entries)
        self.clear_btn.clicked.connect(self.clear_table)

        ##########################
        ### PUPIL CAMERA PANEL ###
        ##########################

        self.record_btn.clicked.connect(self.CMOSthread.trigger_record)
        
        ###################
        ### GRAPH PANEL ###
        ###################

        self.reference_btn.clicked.connect(lambda: self.shutters())

        ###################
        ### MOTOR PANEL ###
        ###################

        loc = self.motor.device.send(60, 0)

        self.home_btn.clicked.connect(self.move_motor_home)
        self.forward_btn.clicked.connect(lambda: self.move_motor_rel(float(self.distance_entry.displayText())))
        self.backward_btn.clicked.connect(lambda: self.move_motor_rel(-float(self.distance_entry.displayText())))
        self.position_btn.clicked.connect(lambda: self.move_motor_abs(float(self.location_entry.displayText())))

        #############################
        ### DATA COLLECTION PANEL ###
        #############################

        self.scan_btn.clicked.connect(lambda: self.EMCCDthread.scan(float(self.start_pos.displayText()),float(self.scan_length.displayText()),int(self.num_frames.displayText())))

        #######################################
        ### VELOCITY AND ACCELERATION PANEL ###
        #######################################

        print "finished showing"

    def update_CMOS_panel(self,camera_data):
        #print "updating CMOS panel"
        #updating cmos panel
        panel_pixmap, coord_pixmap, detected_center, detected_radius = camera_data

        self.cmos_panel.setPixmap(panel_pixmap)
        self.cmos_panel.show()

        self.coord_panel.setPixmap(coord_pixmap)
        self.coord_panel.show()

        #IMPORTANT THAT THESE ARE SET AFTER BOTH PANEL IMAGES ARE SET
        self.detected_center = detected_center
        self.detected_radius = detected_radius
        self.pupil_center_entry.setText(str(self.detected_center))
        self.pupil_radius_entry.setText(str(self.detected_radius))


    def update_EMCCD_panel(self,qImage):
        #print "updating EMCCD panel"
        pixmap = QtGui.QPixmap.fromImage(qImage)
        self.emccd_panel.setPixmap(pixmap)
        self.emccd_panel.show()

    def update_heatmap_panel(self,BS_data=None):
        
        if BS_data is not None:
            pos,BS_profile = BS_data
            self.avg_heatmap.scanned_BS_values[pos] = sum(list(map(lambda profile: profile[1],BS_profile)))/len(BS_profile) #average BS value
            """
            for depth,BS in BS_profile:
                if depth not in self.heatmaps:
                    heatmap = EMCCDthread.HeatMapGraph(50,depth)
                    heatmap.scanned_BS_values[pos] = BS
                    heatmap.plot()
                    self.heatmaps[depth] = heatmap
                    self.slider.setMaximum(len(self.heatmaps)-1)
                else:
                    self.heatmaps[depth].scanned_BS_values[pos] = BS
                    self.heatmaps[depth].plot()
            """
        print "avg heatmap: ", self.avg_heatmap.scanned_BS_values
        self.avg_heatmap.plot()
        self.heatmap_panel.canvas.draw()
        
    """
    def change_heatmap_depth(self,value):

        depth = sorted(self.heatmaps.keys())[value]
        self.heatmap_panel.canvas = FigureCanvasQTAgg(self.heatmaps[depth].fig)

        #self.coord_grid.addWidget(self.heatmap_panel., 4, 0, 3, 4)
        self.heatmap_panel.canvas.draw()
    """
    def update_graph_panel(self,graph_data):

        copied_analyzed_row, brillouin_shift_list = graph_data

        self.graph.fig.clf()
        self.subplot = self.graph.fig.add_subplot(211)
        self.subplot.set_xlabel("Pixel")
        self.subplot.set_ylabel("Counts")

        self.brillouin_plot = self.graph.fig.add_subplot(212)

        self.subplot.scatter(self.graph.x_axis, copied_analyzed_row, s = 1)
        self.brillouin_plot.scatter(np.arange(1, len(brillouin_shift_list)+1), np.array(brillouin_shift_list))

        self.graph_panel.canvas.draw()

    def draw_curve(self,curve_data):
        print "curve_data: ", curve_data
        if len(curve_data) == 2:
            popt, pcov = curve_data
            self.subplot.plot(self.graph.x_axis, EMCCDthread.lorentzian(self.graph.x_axis, *popt), 'r-', label='fit')
        elif len(curve_data) == 4:
            popt, pcov, measured_SD, measured_FSR = curve_data
            self.subplot.plot(self.graph.x_axis, EMCCDthread.lorentzian_reference(self.graph.x_axis, *popt), 'r-', label='fit')
            self.SD_entry.setText(measured_SD)
            self.FSR_entry.setText(measured_FSR)

        self.graph_panel.canvas.draw()

    def take_screenshot(self):
        p = QtGui.QPixmap.grabWindow(self.winId())
        p.save("screenshot.jpg", 'jpg')
        print "shot taken"

    def handle_click(self,event):
        
        if self.set_scan_loc_btn.isChecked():
            x = event.pos().x()
            y = event.pos().y()
            self.CMOSthread.scan_loc = (x,y)
            self.scan_location_x_entry.setText(str(x))
            self.scan_location_y_entry.setText(str(y))
            self.set_scan_loc_btn.setChecked(False)


    def convert_to_pixmap(self,image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channel = image.shape
        bytesPerLine = 3 * width
        qimage = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 
        return QtGui.QPixmap.fromImage(qimage)

    def get_current_ID(self):
        return_id = self.current_ID
        self.current_ID += 1
        return return_id

    def delete_entries(self):
        selected_items = self.scanned_loc_table.selectedItems()
        selected_row_set = set(map(lambda item: item.row(),selected_items))
        sorted_row_set = sorted(list(selected_row_set),reverse=True)

        for row in selected_row_set:
            ID = self.scanned_loc_table.item(row,5).text()
            pos = self.scanned_locations[int(ID)]
            del self.scanned_locations[int(ID)]
            for depth in self.heatmaps:
                heatmap = self.heatmaps[depth]
                if pos in heatmap.scanned_BS_values:
                    del heatmap.scanned_BS_values[pos]
                    heatmap.plot()

        #resets the coord panel to show updated removal of points
        self.CMOSthread.update_coord_panel()
        resized_image = imutils.resize(self.CMOSthread.coord_panel_image, width=self.CMOSthread.coord_panel_image.shape[0]/2)
        coord_pixmap = self.convert_to_pixmap(resized_image)
        self.coord_panel.setPixmap(coord_pixmap)
        self.coord_panel.show()
        self.heatmap_panel.canvas.draw()

        for row in sorted_row_set:
            self.scanned_loc_table.removeRow(row)


    def clear_table(self):
        self.scanned_loc_table.clearContents()
        self.scanned_loc_table.setRowCount(1)
        self.scanned_locations = {}
        self.heatmaps = {-1:self.avg_heatmap}
        self.avg_heatmap.scanned_BS_values = {}
        self.avg_heatmap.plot()

        self.CMOSthread.update_coord_panel()
        resized_image = imutils.resize(self.CMOSthread.coord_panel_image, width=self.CMOSthread.coord_panel_image.shape[0]/2)
        coord_pixmap = self.convert_to_pixmap(resized_image)
        self.coord_panel.setPixmap(coord_pixmap)
        self.coord_panel.show()
        self.heatmap_panel.canvas.draw()


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
        self.location_entry.setText(str(loc.data))

    # moves zaber motor, called on by forwards and backwards buttons
    def move_motor_rel(self,distance):
        self.motor.device.move_rel(distance/0.047625)
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(loc.data*0.047625))

     # moves zaber motor to a set location, called on above
    def move_motor_abs(self,pos):
        self.motor.device.move_abs(pos/0.047625)
        loc = self.motor.device.send(60, 0)
        self.location_entry.setText(str(loc.data*0.047625))

    def set_velocity(self, velocity):
        data = velocity/(9.375*(0.047625/1000))
        self.motor.device.send(42,data)


    def restore_default_params(self):
        self.dp_entry.setText("3.0")
        self.minDist_entry.setText("1000")
        self.param1_entry.setText("1")
        self.param2_entry.setText("15")
        self.range_entry.setText("15")
        self.radius_entry.setText("100")
        self.croppingSize_entry.setText("300")

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
    GUI.show()
    sys.exit(app.exec_())

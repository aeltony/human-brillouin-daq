
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


class EMCCDthread(QtCore.QThread):

    graph_signal = QtCore.pyqtSignal('PyQt_PyObject')
    curve_signal = QtCore.pyqtSignal('PyQt_PyObject')

    def __init__(self,app):
        super(EMCCDthread,self).__init__()

        self.app = app
        self.andor = app.andor
        self.motor = app.motor
        self.graph = app.graph
        self.stop_event = app.stop_event
        self.andor_lock = app.andor_lock
        self.condition = app.condition

        self.image = None
        self.image_andor = None
        self.analyzed_row = np.zeros(80)

        self.export_list = []
        self.brillouin_shift_list = []
        self.scan_ready = False

    def acquire_frame(self):
        with self.andor_lock:
            self.andor.cam.StartAcquisition() 
            data = []                                            
            self.andor.cam.GetAcquiredData(data)

        image_array = np.array(data, dtype = np.uint16)  
        maximum = image_array.max()
        proper_image = np.reshape(image_array, (-1, 512))
        scaled_image = proper_image*(255.0/maximum)
        scaled_image = scaled_image.astype(int)
        scaled_8bit = np.array(scaled_image, dtype = np.uint8)

        loc = np.argmax(scaled_8bit)/512
        left_right = scaled_8bit[loc].argsort()[-10:][::-1]
        left_right.sort()
        mid = int((left_right[0]+left_right[-1])/2)

        cropped = scaled_8bit[loc-7:loc+7, mid-40:mid+40]

        if cropped.size == 0:
            print "CROPPED IS EMPTY"
            cropped = np.zeros((14,80))

        image = imutils.resize(cropped, width=1024)
        image = Image.fromarray(image)
        return image

    def scan(self, start_pos, length, num_steps):

        if self.app.CMOSthread.scan_loc is not None:
            scan_loc = self.app.CMOSthread.scan_loc
            center = self.app.CMOSthread.detected_center
            panel_dim = self.app.coord_panel_image.shape
            panel_center = (panel_dim[1]/2,panel_dim[0]/2)
            relative_coord = (scan_loc[0]-center[0],scan_loc[1]-center[1])

            #updating stored scanned locations
            self.app.CMOSthread.scanned_locations.append(relative_coord)
            self.app.scanned_loc_list.addItem(str(relative_coord))

            #updating coordinate panel
            min_dim = panel_dim[0]/4
            max_dim = 0
            for coord in self.app.CMOSthread.scanned_locations:
                if max(coord) > max_dim:
                    max_dim = max(coord)
                    print "max_dim: ",max_dim
                panel_pos = (coord[0]+panel_center[0],coord[1]+panel_center[1])
                cv2.circle(self.app.coord_panel_image,panel_pos,2,(0,255,255),2)

            """
            #crop panel image
            max_dim += 50 #add a buffer to the edges of image
            if max_dim >= panel_dim[0]/2:
                cropped_image = self.app.coord_panel_image.copy()
                print "no cropping"
            else: 
                print "yes cropping"
                cropping_dim = max(min_dim,max_dim)
                print "cropping dim", cropping_dim
                print "min_dim,max_dim", min_dim, max_dim
                cropped_image = self.app.coord_panel_image.copy()[panel_center[1]-cropping_dim:panel_center[1]+cropping_dim,panel_center[0]-cropping_dim:panel_center[0]+cropping_dim,:]
            """
            resized_image = imutils.resize(self.app.coord_panel_image.copy(), width=panel_dim[0]/2)

            #updating coord panel
            image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
            height, width, channel = image.shape
            bytesPerLine = 3 * width
            coord_image = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 
            pixmap = QtGui.QPixmap.fromImage(coord_image)
            self.app.coord_panel.setPixmap(pixmap)
            self.app.coord_panel.show()


        #move into position to start scan
        self.app.move_motor_abs(start_pos)
        step_size = length // num_steps

        self.export_list.append(self.acquire_frame()) #initial frame

        for i in range(num_steps):

            self.app.move_motor_rel(step_size)           
            
            self.export_list.append(self.acquire_frame())

        #EXPORT
        if len(self.export_list) != 0:
            self.export_list[0].save("data_acquisition/scan.tif",compression="tiff_deflate",save_all=True,append_images=self.export_list[1:]) 
            self.export_list = []
            print "finished exporting as tif"


    def run(self):
        while not self.stop_event.is_set():
            with self.andor_lock:
                self.andor.cam.StartAcquisition() 
                data = []                                            
                self.andor.cam.GetAcquiredData(data)

            image_array = np.array(data, dtype = np.uint16)
            #print "andorLoop"

            maximum = image_array.max()

            proper_image = np.reshape(image_array, (-1, 512))

            scaled_image = proper_image*(255.0/maximum)
            scaled_image = scaled_image.astype(int)
            scaled_8bit = np.array(scaled_image, dtype = np.uint8)

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

            #self.graphLoop()

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

            self.emit(QtCore.SIGNAL('update_EMCCD_panel(PyQt_PyObject)'),qImage)

    #plots graphs, similar to how graphs are plotted on andoru_test.py
    #uses figure so both plots can be shown at same time
    def graphLoop(self):

        copied_analyzed_row = np.array(self.analyzed_row)

        try:
            FSR = float(self.app.FSR_entry.displayText())
            SD = float(self.app.SD_entry.displayText())
            
            if not self.reference_btn.isChecked():
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
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

class Popup(QtGui.QWidget):

    def __init__(self,CMOSthread):
        super(Popup,self).__init__()

        self.CMOSthread = CMOSthread
        self.qImage_snapshot = CMOSthread.qImage

        grid = QtGui.QGridLayout()
        self.setLayout(grid)

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
        if self.click_pos is not None and self.release_pos is not None:
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

        self.coords = False
        self.detected_radius = None
        self.detected_center = None
        self.scan_loc = None

    def set_coordinates(self):
        self.coords = not self.coords

    def apply_parameters(self):
        self.medianBlur = int(self.app.blur_entry.displayText())
        self.dp = float(self.app.dp_entry.displayText())
        self.minDist = int(self.app.minDist_entry.displayText())
        self.param1 = int(self.app.param1_entry.displayText())
        self.param2 = int(self.app.param2_entry.displayText())
        self.radius_range = int(self.app.range_entry.displayText())
        self.expected_pupil_radius = int(self.app.radius_entry.displayText())
        print "expected Pupil radius", self.expected_pupil_radius

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

            if self.scan_loc is not None:
                size = 20
                dim = plain_image.shape
                cv2.line(plain_image,(self.scan_loc[0],min(self.scan_loc[1]+size,dim[0])),(self.scan_loc[0],max(self.scan_loc[1]-size,0)),(0,255,0),1)
                cv2.line(plain_image,(min(self.scan_loc[0]+size,dim[1]),self.scan_loc[1]),(max(self.scan_loc[0]-size,0),self.scan_loc[1]),(0,255,0),1)


            if self.medianBlur is None or self.dp is None or self.minDist is None or self.param1 is None or self.param2 is None or self.radius_range is None or self.expected_pupil_radius is None:
                pupil_data = [plain_image,None,None]
            else:
                #start_time = time.time()
                pupil_data = ht.detect_pupil_frame(plain_image,self.medianBlur,self.dp,self.minDist,self.param1,self.param2,self.radius_range,self.expected_pupil_radius,self.coords,self.app.scanned_locations)
                #print "HT run time: ",time.time() - start_time

            if self.app.record_btn.isChecked(): # extra check to cover for out incorrect ordering case

                self.pupil_video_frames.append(pupil_data[0].copy())
                self.pupil_data_list.append((pupil_data[1],pupil_data[2]))
                print "added frame to list"
            

            # convets image to form used by pyqt
            image = cv2.cvtColor(pupil_data[0].copy(), cv2.COLOR_BGR2RGB)
            height, width, channel = image.shape
            bytesPerLine = 3 * width
            qImage = QtGui.QImage(image.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) 

            self.qImage = qImage

            #added after sending next frame because scan might be taking pupil center from next frame
            self.detected_center = pupil_data[1]
            self.detected_radius = pupil_data[2]

            self.emit(QtCore.SIGNAL('update_CMOS_panel(PyQt_PyObject)'),self.qImage)


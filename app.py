
from PIL import Image
from PIL import ImageTk
import Tkinter as tki
import threading
import datetime
import imutils
import cv2
import numpy as np
import time
import math
import os
import sys 
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



class App(threading.Thread):
    def __init__(self, outputPath):

        threading.Thread.__init__(self)
        self.start()
        self.lock = threading.Lock()
        self.condition = threading.Condition()

        self.root = tki.Tk()
        # initiallize and access cameras, motors and graphs
        self.mako = device_init.Mako_Camera()
        self.andor = device_init.Andor_Camera()
        self.motor = device_init.Motor()
        self.graph = Graph()


        self.outputPath = outputPath
        self.frame = self.mako.camera.getFrame()
        self.frame.announceFrame()
        self.image = None
        self.image_andor = None
        self.analyzed_row = np.zeros(80)

        self.brillouin_shift_list = []
        self.PlasticBS =  9.6051
        self.WaterBS = 5.1157

        self.thread = None
        self.thread2 = None
        self.stopEvent = None

        # initialize the root window and image panel
        #CMOS camera panel
        self.panelA = None
        #EMCCD camera panel
        self.panelB = None
        #where graphs are drwan
        self.canvas = FigureCanvasTkAgg(self.graph.fig, master = self.root)

        # GUI constants
        self.record = False
        self.pupil_video_writer = None
        self.andor_image_list = []
        self.scan_ready = False
        self.andor_export_image = None

        #button will take the current frame and save it to file
        snapshot_btn = tki.Button(self.root, text="Picture", command=self.takeSnapshot)
        snapshot_btn.grid(row = 3, column = 0, sticky = "w") 

        record_btn = tki.Button(self.root, text="Record", command=self.triggerRecord)
        record_btn.grid(row=3, column=1, sticky="w")

        #reference button, shutter_state also used to detemine graph fitting 
        #tied to function for changing shutter states
        self.shutter_state = tki.IntVar()
        reference_btn = tki.Checkbutton(self.root, text = "Reference", variable = self.shutter_state, command = self.shutters, indicatoron = 0)
        reference_btn.grid(row = 3, column = 3, sticky = "w") 

        #Shows current FSR on interface
        FSR_label = tki.Label(self.root, text = "FSR ").grid(row = 3, column = 3, sticky = "e")
        self.FSR = tki.DoubleVar()
        self.FSR.set(16.2566)
        FSR_entry = tki.Entry(self.root, textvariable = self.FSR)
        FSR_entry.grid(row = 3, column = 4, sticky = "w")

        #Shows current SD on interface
        SD_label = tki.Label(self.root, text = "SD ").grid(row = 3, column = 4, sticky = "e")
        self.SD = tki.DoubleVar()
        self.SD.set(0.14288)
        SD_entry = tki.Entry(self.root, textvariable = self.SD)
        SD_entry.grid(row = 3, column = 5, sticky = "w")


        ### MOTOR PANEL ###

        motor_label = tki.Label(self.root, text = "Motor Control")
        motor_label.grid(row = 4, column = 0, pady=10, sticky = "sw")

        #home button for motor
        home_btn = tki.Button(self.root, text = "Home", command = self.motor.device.home)
        home_btn.grid(row = 5, column = 0, sticky = "nw")

       	#distance and location variable for motor
        distance_var = tki.IntVar()
        distance_var.set(0)
        self.location_var = tki.IntVar()
        current_location = self.motor.device.send(60, 0)
        self.location_var.set(current_location.data)

   		#shows distance to move
        distance_label = tki.Label(self.root, text = "Distance to Move").grid(row = 4, column = 1, sticky = "sw")
        distance_entry = tki.Entry(self.root, textvariable = distance_var)
        distance_entry.grid(row = 5, column = 1, sticky = "nw")

        #moves motor forward by given distance from above
        forward_button = tki.Button(self.root, text = "Forward", command = lambda: self.move_motor_relative(distance_var.get()))
        forward_button.grid(row = 5, column = 2, sticky = "nw")

        #moves motor backwards by given distance from above
        back_button = tki.Button(self.root, text = "Backwards", command = lambda: self.move_motor_relative(-distance_var.get()))
        back_button.grid(row = 5, column = 3, sticky = "nw")

        #shows current location of motor on rails
        location_label = tki.Label(self.root, text = "Position").grid(row = 4, column = 4, sticky = "sw")
        location_entry = tki.Entry(self.root, textvariable = self.location_var)
        location_entry.grid(row = 5, column = 4, sticky = "nw")

        #can enter a different location above and move motor to entered location
        position_button = tki.Button(self.root, text =  "Move to Position", command = lambda: self.move_motor_abs(self.location_var.get()) )
        position_button.grid(row = 5, column = 5, sticky = "nw")


        # Data collection: 
        start_pos = tki.IntVar()
        start_pos.set(0)
        start_pos_label = tki.Label(self.root, text="Start(um)").grid(row = 6, column = 0)
        start_pos_entry = tki.Entry(self.root, textvariable = start_pos)
        start_pos_entry.grid(row = 7, column = 0)
        
        num_frames = tki.IntVar()
        num_frames.set(0)
        num_frames_label = tki.Label(self.root, text="#Frames").grid(row = 6, column = 1)
        num_frames_entry = tki.Entry(self.root, textvariable = num_frames)
        num_frames_entry.grid(row = 7, column = 1)
        
        scan_length = tki.IntVar()
        scan_length.set(0)
        scan_length_label = tki.Label(self.root, text="Length(um)").grid(row = 6, column = 2)
        scan_length_entry = tki.Entry(self.root, textvariable = scan_length)
        scan_length_entry.grid(row = 7, column = 2)

       	scan_btn = tki.Button(self.root, text="Start Scan", command = lambda: self.slice_routine(start_pos.get(),scan_length.get(),num_frames.get()))
        scan_btn.grid(row = 7, column = 3)



        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread2 = threading.Thread(target=self.andorLoop, args=())
        self.thread.start()
        self.thread2.start()

        # set a callback to handle when the window is closed
        self.root.wm_title("Pupil")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)
        

    #Loop for thread for CMOS camera - almost exact same as mako_pupil.py
    def videoLoop(self):
        self.mako.camera.startCapture()
        self.mako.camera.runFeatureCommand('AcquisitionStart')
        self.frame.queueFrameCapture()
    
        frame_number = 0
        pupil_data_file = open('data_acquisition/pupil_data.txt','w+')


        while not self.stopEvent.is_set():
            # self.root.update()
            self.frame.waitFrameCapture(1000)
            self.frame.queueFrameCapture()
            imgData = self.frame.getBufferByteData()
            image = np.ndarray(buffer = imgData,
                           dtype = np.uint8,
                           shape = (self.frame.height,self.frame.width))    
            
            
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            pupil_data = ht.detect_pupil_frame(image)

            if pupil_data[1] is None or pupil_data[2] is None: 
                pupil_data_file.write("Frame: %d No pupil detected!\n" % frame_number)
            else: 
                pupil_data_file.write("Frame: %d Pupil Center: %s Pupil Radius: %d\n" % (frame_number,pupil_data[1],pupil_data[2]))
            
            
            drawing = pupil_data[0]


            if self.record is True:
                if self.pupil_video_writer is None:
                    self.pupil_video_writer = skv.FFmpegWriter('data_acquisition/pupil_video.avi',outputdict={
                    '-vcodec':'libx264',
                    '-b':'30000000',
                    '-vf':'setpts=4*PTS'
                    })
                    print "made a new video writer!"
                else:
                    self.pupil_video_writer.writeFrame(drawing)
                    print "writing to the video"
            elif self.record is False and self.pupil_video_writer is not None:
                self.pupil_video_writer.close()
                self.pupil_video_writer = None
                print "closed the writer"



            self.image = drawing
            # convets image to form used by tkinter
            image = cv2.cvtColor(drawing, cv2.COLOR_BGR2RGB)
            image = imutils.resize(image, width=1024)
            image = Image.fromarray(image)
            image = ImageTk.PhotoImage(image)
    	
    		#for front end 
            # if the panel is not None, we need to initialize it
            if self.panelA is None:
                self.panelA = tki.Label(image=image)
                self.panelA.image = image
                self.panelA.grid(row = 0, column = 0, columnspan = 6, rowspan = 3) #pack(side="left", padx=10, pady=10)
    
            # otherwise, simply update the panel
            else:
                self.panelA.configure(image=image)
                self.panelA.image = image

        pupil_data_file.close()


     #almost exactly same as andor_test.py 
     #graphing is called in a function graphLoop()
    def andorLoop(self):
        while not self.stopEvent.is_set():
            self.andor.cam.StartAcquisition() 
            data = []                                            
            self.andor.cam.GetAcquiredData(data)
            while data == []:
                continue

            image_array = np.array(data, dtype = np.uint16)
            maximum = image_array.max()


            graph_data = list(data)  
            graph_array = np.array(graph_data, dtype = np.uint16)
            reshaped_graph = np.reshape(graph_array, (-1, 512))

            proper_image = np.reshape(image_array, (-1, 512))

            if self.scan_ready: 
                self.condition.acquire()
                print "lock acquired"
                self.andor_export_image = Image.fromarray(proper_image)
                self.scan_ready = False
                self.condition.notifyAll()
                print "threads notified"
                self.condition.release()
                print "lock released"



            scaled_image = proper_image*(255.0/maximum)
            scaled_image = scaled_image.astype(int)
            scaled_8bit= np.array(scaled_image, dtype = np.uint8)
           

            loc = np.argmax(scaled_8bit)/512
            left_right = scaled_8bit[loc].argsort()[-10:][::-1]
            left_right.sort()
            mid = int((left_right[0]+left_right[-1])/2)

        
            self.analyzed_row = reshaped_graph[loc][mid-40:mid+40]


            cropped = scaled_8bit[loc-7:loc+7, mid-40:mid+40]
            #self.graphLoop()
            
            (h, w)= cropped.shape[:2]
            if w <= 0 or h <= 0:
                continue

            self.image_andor = cropped

            #if not take_scan:
            #    self.andor_image_list.append(self.image_andor)
            #    take_scan = True
            

            image = imutils.resize(cropped, width=1024)
            image = Image.fromarray(image)
            

            #self.andor_image_list.append(image)

            image = ImageTk.PhotoImage(image)

            print "about to update andor"

            # if the panel is not None, we need to initialize it
            if self.panelB is None:
                self.panelB = tki.Label(image=image)
                self.panelB.image = image
                self.panelB.grid(row = 0, column = 6, columnspan = 3) #pack(side="left", padx=10, pady=10)

            # otherwise, simply update the panel
            else:
                try:
                    self.panelB.configure(image=image)
                    self.panelB.image = image
                except:
                    continue
            
    #similar to shutters.py, called on by reference button 
    def shutters(self, close = False):
        dll = WinDLL("C:\\Program Files\\quad-shutter\\Quad Shutter dll and docs\\x64\\PiUsb")
        c2 = c_int()
        c4 = c_int()
        usb312 = dll.piConnectShutter(byref(c2), 312)
        usb314 = dll.piConnectShutter(byref(c4), 314)

        with self.lock:
            state = self.shutter_state.get()
        if state == 1 and close == False:

            dll.piSetShutterState(0, usb312)
            dll.piSetShutterState(1, usb314)
        else:
            dll.piSetShutterState(1, usb312)
            dll.piSetShutterState(0, usb314)

        dll.piDisconnectShutter(usb312)
        dll.piDisconnectShutter(usb314)

     #plots graphs, similar to how graphs are plotted on andoru_test.py
     #uses figure so both plots can be shown at same time
    def graphLoop(self):
        self.graph.fig.clf()
        subplot = self.graph.fig.add_subplot(211)
        subplot.set_xlabel("Pixel")
        subplot.set_ylabel("Counts")

        brillouin_plot = self.graph.fig.add_subplot(212)


        copied_analyzed_row = np.array(self.analyzed_row)

        with self.lock:
            state = self.shutter_state.get() 

        try:
            
            if state == 0:
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
                BS = (self.FSR.get() - delta_peaks*self.SD.get())/2
                length_bs = len(self.brillouin_shift_list)
                if length_bs >= 100:
                    self.brillouin_shift_list = []
                self.brillouin_shift_list.append(BS)
                length_bs +=1

                
                
                popt, pcov = curve_fit(lorentzian, self.graph.x_axis, copied_analyzed_row, p0 = np.array([gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, 100]))
                subplot.plot(self.graph.x_axis, lorentzian(self.graph.x_axis, *popt), 'r-', label='fit')
               
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
                subplot.plot(self.graph.x_axis, lorentzian_reference(self.graph.x_axis, *popt), 'r-', label='fit')
                measured_SD = (2*self.PlasticBS - 2*self.WaterBS) / ((x0_4 - x0_1) + (x0_3 - x0_2))
                measured_FSR = 2*self.PlasticBS - measured_SD*(x0_3 - x0_2)
                self.SD.set(measured_SD)
                self.FSR.set(measured_FSR)

        except:
            print "Graph fitting failed"
            pass


            
        subplot.scatter(self.graph.x_axis, copied_analyzed_row, s = 1)
        brillouin_plot.scatter(np.arange(1, len(self.brillouin_shift_list)+1), np.array(self.brillouin_shift_list))

        self.canvas.show()
        self.canvas.get_tk_widgalet().grid(row = 1, column = 6, columnspan = 3, rowspan = 6)    #pack(side = "right")

    # moves zabor motor, called on by forwars and backwards buttons
    def move_motor_relative(self, distance):
        self.motor.device.move_rel(distance)
        loc = self.motor.device.send(60, 0)
        self.location_var.set(loc.data)

     # moves zabor motor to a set location, called on above
    def move_motor_abs(self, location):
        self.motor.device.move_abs(location)
        loc = self.motor.device.send(60, 0)
        self.location_var.set(loc.data)


    def slice_routine(self, start_pos, length, num_steps):
        self.motor.device.move_abs(start_pos)
        step_size = length // num_steps
        imlist = []
        for i in range(num_steps):
            self.condition.acquire()

            self.motor.device.move_rel(step_size)            
            print "moving.. "

            self.scan_ready = True
            self.condition.wait()

            print "adding images.."
            imlist.append(self.andor_export_image)

        print "length of imlist: ",len(imlist)
        # Save images to tif file 
        if len(imlist) != 0:
            imlist[0].save("data_acquisition/scan.tif",compression="tiff_deflate",save_all=True,append_images=imlist[1:]) 
            print "finished exporting as tif"

    def takeSnapshot(self):
        # grab the current timestamp and use it to construct the
        # output path
        ts = datetime.datetime.now()
        filename = "{}.jpg".format(ts.strftime("%Y-%m-%d_%H-%M-%S"))
        p = os.path.sep.join((self.outputPath, filename))
 
        # save the file

        cv2.imwrite(p, self.image.copy())

        print("[INFO] saved {}".format(filename))

    def triggerRecord(self):
        self.record = not self.record


    # what happens when you exit out of application window
    # release connections to devices, closes application
    def onClose(self):
        self.stopEvent.set()
        self.mako.camera.runFeatureCommand('AcquisitionStop')
        self.mako.camera.endCapture()
        self.mako.camera.revokeAllFrames()
        self.mako.vimba.shutdown()
        self.motor.port.close()
        self.shutters(close = True)
        self.root.quit()
        self.root.destroy()




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
        

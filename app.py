
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
import traceback
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



class App(threading.Thread):
    def __init__(self, outputPath):

        threading.Thread.__init__(self)
        self.start()

        self.root = tki.Tk()


        #threads and locks
        self.thread = None
        self.thread2 = None
        self.stopEvent = None
        
        self.lock = threading.Lock()
        self.record_lock = threading.Lock()
        self.condition = threading.Condition()
        self.stopEvent = threading.Event()


        # initialize and access cameras, motors and graphs
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

        # initialize the root window and image panel
        #CMOS camera panel
        self.panelA = None
        #EMCCD camera panel
        self.panelB = None
        #where graphs are drawn
        self.canvas = FigureCanvasTkAgg(self.graph.fig, master = self.root)

        # GUI constants
        self.pupil_video_frames = []
        self.pupil_data_list = []
        self.andor_image_list = []
        self.scan_ready = False
        self.andor_export_image = None
        self.click_pos = None
        self.release_pos = None
        self.expected_pupil_radius = 0


        ##########################
        ### PUPIL CAMERA PANEL ###
        ##########################

        #button will take the current frame and save it to file
        snapshot_btn = tki.Button(self.root, text="Picture", command=self.takeSnapshot)
        snapshot_btn.grid(row = 3, column = 0, sticky = "w") 

        #button to start recording pupil cam and resulting pupil detection data
        self.record = tki.IntVar()
        record_btn = tki.Checkbutton(self.root, text="Record", variable = self.record, command=self.triggerRecord, indicatoron = 0)
        record_btn.grid(row=3, column=1, sticky="w")


        ###################
        ### GRAPH PANEL ###
        ###################

        #reference button, shutter_state also used to determine graph fitting
        self.shutter_state = tki.IntVar()
        reference_btn = tki.Checkbutton(self.root, text = "Reference", variable = self.shutter_state, command = self.shutters, indicatoron = 0)
        reference_btn.grid(row = 3, column = 2, sticky = "w") 

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


        ###################
        ### MOTOR PANEL ###
        ###################

        motor_label = tki.Label(self.root, text = "Motor Control")
        motor_label.grid(row = 4, column = 0, pady=10, sticky = "sw")

        #home button for motor
        home_btn = tki.Button(self.root, text = "Home", command = lambda: self.move_motor_home())
        home_btn.grid(row = 5, column = 0, sticky = "nw")

   		#entry for user to input distance to move motor forward or backward
        distance_var = tki.IntVar()
        distance_var.set(0)
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
        self.location_var = tki.IntVar()
        current_location = self.motor.device.send(60, 0)
        self.location_var.set(int(current_location.data*3.072))
        location_label = tki.Label(self.root, text = "Position").grid(row = 4, column = 4, sticky = "sw")
        location_entry = tki.Entry(self.root, textvariable = self.location_var)
        location_entry.grid(row = 5, column = 4, sticky = "nw")

        #can enter a different location above and move motor to entered location
        position_button = tki.Button(self.root, text =  "Move to Position", command = lambda: self.move_motor_abs(self.location_var.get()))
        position_button.grid(row = 5, column = 5, sticky = "nw")


        #############################
        ### DATA COLLECTION PANEL ###
        #############################

        #entry for motor start position
        start_pos = tki.IntVar()
        start_pos.set(0)
        start_pos_label = tki.Label(self.root, text="Start(um)").grid(row = 6, column = 0)
        start_pos_entry = tki.Entry(self.root, textvariable = start_pos)
        start_pos_entry.grid(row = 7, column = 0)
        
        #entry for number of pictures to take during scan
        num_frames = tki.IntVar()
        num_frames.set(0)
        num_frames_label = tki.Label(self.root, text="#Frames").grid(row = 6, column = 1)
        num_frames_entry = tki.Entry(self.root, textvariable = num_frames)
        num_frames_entry.grid(row = 7, column = 1)
        
        #entry for how far motor moves in total
        scan_length = tki.IntVar()
        scan_length.set(0)
        scan_length_label = tki.Label(self.root, text="Length(um)").grid(row = 6, column = 2)
        scan_length_entry = tki.Entry(self.root, textvariable = scan_length)
        scan_length_entry.grid(row = 7, column = 2)

        #button to start scan
       	scan_btn = tki.Button(self.root, text="Start Scan", command = lambda: self.slice_routine(start_pos.get(),scan_length.get(),num_frames.get()))
        scan_btn.grid(row = 7, column = 3)


        ##########################################
        ### VELOCITY AND ACCELERATION CONTROLS ###
        ##########################################

        #controls to enter and change velocity of motor
        velocity_var = tki.IntVar()
        velocity_var.set(0)
        velocity_label = tki.Label(self.root, text="Velocity").grid(row = 6, column = 4)
        velocity_entry = tki.Entry(self.root, textvariable = velocity_var)
        velocity_entry.grid(row = 7, column = 4)

        velocity_btn = tki.Button(self.root, text="Change velocity", command = lambda: self.set_velocity(velocity_var.get()))
        velocity_btn.grid(row = 7, column = 5)


        #controls to enter and change acceleration of motor
        #TODO


        #initialize and start threads
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread2 = threading.Thread(target=self.andorLoop, args=())
        self.thread.start()
        self.thread2.start()

        #set a callback to handle when the window is closed
        self.root.wm_title("Brillouin Scan Interface")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)


        #loop to update CMOS, EMCCD, and graph panels in GUI
        #All changes to GUI must happen in one thread so to achieve this: 
        #videoLoop and andorLoop adds CMOS, EMCCD and graph images to a thread-safe queue
        #This main Tkinter thread dequeues images one by one and updates their respective panels on the GUI
        self.queue = Queue.Queue()
        self.update_root()

    #Dequeues items from self.queue and updates respective widgets in the GUI
    def update_root(self):

        while not self.stopEvent.is_set():
            try:
                destination, item = self.queue.get(timeout=0.1)

                if destination == "panelA":
                    if self.panelA is None:
                        self.panelA = tki.Label(self.root,image=item)
                        self.panelA.image = item
                        self.panelA.grid(row = 0, column = 0, columnspan = 6, rowspan = 3) #pack(side="left", padx=10, pady=10)
                    else:
                        self.panelA.configure(image=item)
                        self.panelA.image = item

                elif destination == "panelB":
                    if self.panelB is None:
                        self.panelB = tki.Label(self.root,image=item)
                        self.panelB.grid_propagate(0)
                        self.panelB.image = item
                        self.panelB.grid(row = 0, column = 6, columnspan = 3, sticky = "n") #pack(side="left", padx=10, pady=10)

                        self.panelB.configure(bg="red")
                    else:
                        self.panelB.configure(image=item)
                        self.panelB.grid_propagate(0)
                        self.panelB.image = item
                
                elif destination == "scatter":
                    self.graph.fig.clf()
                    subplot = self.graph.fig.add_subplot(211)
                    subplot.set_xlabel("Pixel")
                    subplot.set_ylabel("Counts")

                    brillouin_plot = self.graph.fig.add_subplot(212)

                    copied_analyzed_row,brillouin_shift_list = item
                    subplot.scatter(self.graph.x_axis, copied_analyzed_row, s = 1)
                    brillouin_plot.scatter(np.arange(1, len(brillouin_shift_list)+1), np.array(brillouin_shift_list))

                    self.canvas.show()
                    self.canvas.get_tk_widget().grid(row = 1, column = 6, columnspan = 3, rowspan = 6)    #pack(side = "right")
                
                #elif destination == "plot":
                #    popt = item
                #    subplot.plot(self.graph.x_axis, lorentzian(self.graph.x_axis, *popt), 'r-', label='fit')

            except:
                #print "break"
                break
        self.root.after(300,self.update_root)


    #Loop for thread for CMOS camera - almost exact same as mako_pupil.py
    def videoLoop(self):
        self.mako.camera.startCapture()
        self.mako.camera.runFeatureCommand('AcquisitionStart')
        self.frame.queueFrameCapture()


        while not self.stopEvent.is_set():
            #print "videoLoop"
            # self.root.update()
            self.frame.waitFrameCapture(1000)
            self.frame.queueFrameCapture()
            imgData = self.frame.getBufferByteData()
            image = np.ndarray(buffer = imgData,
                           dtype = np.uint8,
                           shape = (self.frame.height,self.frame.width))    
            
            
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

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
                    

            self.image = pupil_data[0]
            # convets image to form used by tkinter
            image = cv2.cvtColor(pupil_data[0].copy(), cv2.COLOR_BGR2RGB)
            image = imutils.resize(image, width=1024)
            image = Image.fromarray(image)
            image = ImageTk.PhotoImage(image)
    	
            self.queue.put(("panelA",image))


     #almost exactly same as andor_test.py 
     #graphing is called in a function graphLoop()
    def andorLoop(self):
        while not self.stopEvent.is_set():
            #print "andorLoop"
            self.andor.cam.StartAcquisition() 
            data = []                                            
            self.andor.cam.GetAcquiredData(data)
            """
            while data == []:
                continue
            """
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
                #print "andorloop lock acquired"
                self.andor_export_image = Image.fromarray(scaled_8bit)
                self.scan_ready = False
                self.condition.notifyAll()
                #print "threads notified"
                self.condition.release()
                #print "lock released"

            loc = np.argmax(scaled_8bit)/512
            left_right = scaled_8bit[loc].argsort()[-10:][::-1]
            left_right.sort()
            mid = int((left_right[0]+left_right[-1])/2)

        
            self.analyzed_row = reshaped_graph[loc][mid-40:mid+40]


            cropped = scaled_8bit[loc-7:loc+7, mid-40:mid+40]
            self.graphLoop()
            print(cropped.shape,loc)
            (h, w)= cropped.shape[:2]
            if w <= 0 or h <= 0:
                continue

            self.image_andor = cropped

            image = imutils.resize(self.image_andor, width=1024)
            image = Image.fromarray(image)
            

            image = ImageTk.PhotoImage(image)


            self.queue.put(("panelB",image))

            
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
        #self.graph.fig.clf()
        #subplot = self.graph.fig.add_subplot(211)
        #subplot.set_xlabel("Pixel")
        #subplot.set_ylabel("Counts")

        #brillouin_plot = self.graph.fig.add_subplot(212)


        copied_analyzed_row = np.array(self.analyzed_row)
        print len(copied_analyzed_row)

        with self.lock:
            state = self.shutter_state.get() 

        try:
            
            if state == 0:
                constant_1 = np.amax(copied_analyzed_row[:40])
                constant_2 = np.amax(copied_analyzed_row[40:])
                x0_1 = np.argmax(copied_analyzed_row[:40])
                x0_2 = np.argmax(copied_analyzed_row[40:])+40
                gamma_1 = None
                gamma_2 = None
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

                
                if gamma_1 is not None and gamma_2 is not None:
                    popt, pcov = curve_fit(lorentzian, self.graph.x_axis, copied_analyzed_row, p0 = np.array([gamma_1, x0_1, constant_1, gamma_2, x0_2, constant_2, 100]))
                    self.queue.put(("plot",popt.copy()))
                    #subplot.plot(self.graph.x_axis, lorentzian(self.graph.x_axis, *popt), 'r-', label='fit')
               
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
                self.queue.put(("plot",popt.copy()))
                #subplot.plot(self.graph.x_axis, lorentzian_reference(self.graph.x_axis, *popt), 'r-', label='fit')
                measured_SD = (2*self.PlasticBS - 2*self.WaterBS) / ((x0_4 - x0_1) + (x0_3 - x0_2))
                measured_FSR = 2*self.PlasticBS - measured_SD*(x0_3 - x0_2)
                self.SD.set(measured_SD)
                self.FSR.set(measured_FSR)

        except Exception as e:
            print "Error is: ",str(e)
            print "Stack trace: ", traceback.format_exc()
            pass


        self.queue.put(("scatter",(copied_analyzed_row.copy(),self.brillouin_shift_list[:])))
        #subplot.scatter(self.graph.x_axis, copied_analyzed_row, s = 1)
        #brillouin_plot.scatter(np.arange(1, len(self.brillouin_shift_list)+1), np.array(self.brillouin_shift_list))


    # moves zaber motor to home position
    def move_motor_home(self):
        self.motor.device.home()
        loc = self.motor.device.send(60,0)
        self.location_var.set(int(loc.data*3.072))

    # moves zaber motor, called on by forwards and backwards buttons
    def move_motor_relative(self, distance):
        self.motor.device.move_rel(int(distance/3.072))
        loc = self.motor.device.send(60, 0)
        self.location_var.set(int(loc.data*3.072))

     # moves zaber motor to a set location, called on above
    def move_motor_abs(self, location):
        self.motor.device.move_abs(int(location/3.072))
        loc = self.motor.device.send(60, 0)
        self.location_var.set(int(loc.data*3.072))

    def set_velocity(self, velocity):
        self.motor.device.send(42,velocity)

    def slice_routine(self, start_pos, length, num_steps):
        self.move_motor_abs(start_pos)
        step_size = length // num_steps
        imlist = []
        for i in range(num_steps):
            self.condition.acquire()
            #print "slice_routine acquired lock"

            self.move_motor_relative(step_size)            
            #print "moving.. "
            print self.location_var.get()
            self.scan_ready = True
            #print "waiting.."
            self.condition.wait()

            imlist.append(self.andor_export_image)
            #print "adding images.."

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
        if self.record.get() == 0:
            with self.record_lock:

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


    def onClick(self,event):
        self.click_pos = (event.x,event.y)
        self.release_pos = None
        print self.click_pos

    def onRelease(self,event):
        self.release_pos = (event.x,event.y)
        print self.release_pos


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
        

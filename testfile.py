from PyQt4 import QtGui,QtCore
import numpy as np

import random

# graphing imports
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from scipy.optimize import curve_fit
from scipy.interpolate import griddata
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


import sys

        
class App(QtGui.QWidget):

    def __init__(self):
        super(App,self).__init__()
        
        self.grid = QtGui.QGridLayout()
        self.setLayout(self.grid)

        self.heatmap = HeatMapGraph(25)

        self.heatmap_panel = FigureCanvasQTAgg(self.heatmap.fig)
        self.heatmap.ax = self.heatmap.fig.add_subplot(111, projection='3d')


        #self.scanned_BS_values = np.zeros(self.heatmap.X.shape)
        #self.scanned_BS_values.fill(0.5)


        dict_template = [((x,y),0.5) for x in range(-600,601,100) for y in range(-600,601,100)]
        self.scanned_BS_values = dict(dict_template)


        points = self.scanned_BS_values.keys()
        values = [self.scanned_BS_values[key] for key in points]

        grid = griddata(points,values,(self.heatmap.X,self.heatmap.Y),method="cubic")

        img = self.heatmap.ax.plot_surface(self.heatmap.X, self.heatmap.Y, grid, rstride=1, cstride=1, cmap=cm.rainbow, antialiased=False)

        self.heatmap.set_canvas(self.heatmap_panel)
        Axes3D.mouse_init(self.heatmap.ax)

        self.heatmap.fig.suptitle("Brillouin Shift Frequency Map",  fontsize=12)
        self.heatmap.ax.set_xlabel('x (pixels)')
        self.heatmap.ax.set_ylabel('y (pixels)')
        self.heatmap.ax.set_zlabel('Average Brillouin shift (GHz)')
        self.heatmap.fig.colorbar(img, shrink=0.5, aspect=10)


        btn = QtGui.QPushButton("add point",self)
        btn.clicked.connect(self.add_point)

        screenshot_btn = QtGui.QPushButton("take screenshot",self)
        screenshot_btn.clicked.connect(self.take_screenshot)

        self.grid.addWidget(self.heatmap_panel,0,0,4,4)
        self.grid.addWidget(btn,0,4,1,1)
        self.grid.addWidget(screenshot_btn,1,4,1,1)

        self.heatmap_panel.draw()

        filename = QtGui.QFileDialog.getSaveFileName(self, 'Open File', '/')
        print filename


        self.setWindowTitle("Brillouin Scan Interface")
        self.move(50,50)
        self.show()
    
    def take_screenshot(self):
        p = QtGui.QPixmap.grabWindow(self.winId())
        p.save("pupil_videos/screenshot.jpg", 'jpg')
        print "shot taken"


    def add_point(self):
        value = random.random()
        
        x = random.randrange(-600,601,25)
        y = random.randrange(-600,601,25)
        self.scanned_BS_values[(x,y)] = value

        print x, y, value

        self.heatmap.fig.clf()
        self.heatmap.ax = self.heatmap.fig.add_subplot(111, projection='3d')

        self.heatmap.fig.suptitle("Brillouin Shift Frequency 3D Map",  fontsize=12)
        self.heatmap.ax.set_xlabel('x (pixels)')
        self.heatmap.ax.set_ylabel('y (pixels)')
        self.heatmap.ax.set_zlabel('Average Brillouin shift (GHz)')

        """
        mrange = [-600,-500,-400,-300,-200,-100,0,100,200,300,400,500,600]
        initial_points = np.array(np.meshgrid(mrange,mrange)).T.reshape(-1,2)
        values = np.array(list(map(lambda point: self.scanned_BS_values[point],initial_points))).T.reshape(-1)
        """
        points = self.scanned_BS_values.keys()
        values = [self.scanned_BS_values[key] for key in points]

        grid = griddata(points,values,(self.heatmap.X,self.heatmap.Y),method="cubic")
        

        self.heatmap.ax.plot_surface(self.heatmap.X, self.heatmap.Y, grid, rstride=1, cstride=1, cmap=cm.rainbow)

        Axes3D.mouse_init(self.heatmap.ax)
        self.heatmap_panel.draw()



class HeatMapGraph:
    def __init__(self,resolution):
        self.fig = Figure(figsize = (5,5), dpi = 100)
        self.ax = None

        X = np.arange(-600, 600, resolution)
        Y = np.arange(-600, 600, resolution)
        self.X, self.Y = np.meshgrid(X, Y)

    def set_canvas(self,canvas):
        self.fig.set_canvas(canvas)


if __name__ == '__main__':
	import matplotlib.pyplot as plt
	import numpy as np

	x, y, z = np.random.random((3, 100))

	plt.ion()

	fig, ax = plt.subplots()
	scat = ax.scatter(x, y, c=z, s=200)

	for _ in range(20):
	    # Change the colors...
	    #scat.set_array(np.random.random(100))
	    # Change the x,y positions. This expects a _single_ 2xN, 2D array
	    scat.set_offsets(np.random.random((2,100)))
	    fig.canvas.draw()
	    

    





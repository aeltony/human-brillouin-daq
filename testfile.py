from PyQt4.QtGui import *
from PyQt4.QtCore import *
import numpy as np
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

        
class App(QWidget):

    def __init__(self):
        super(App,self).__init__()
        
        self.grid = QGridLayout()
        self.setLayout(self.grid)

        self.heatmap = HeatMapGraph(25)

        self.heatmap_panel = FigureCanvasQTAgg(self.heatmap.fig)
        self.heatmap.ax = self.heatmap.fig.add_subplot(111, projection='3d')

        self.scanned_BS_values = np.zeros(self.heatmap.X.shape)
        self.scanned_BS_values.fill(6)

        img = self.heatmap.ax.plot_surface(self.heatmap.X, self.heatmap.Y, self.scanned_BS_values, rstride=1, cstride=1, cmap=cm.rainbow, antialiased=False)

        self.heatmap.set_canvas(self.heatmap_panel)
        Axes3D.mouse_init(self.heatmap.ax)

        self.grid.addWidget(self.heatmap_panel,0,0,4,4)


        self.heatmap.fig.suptitle("Brillouin Shift Frequency 3D Map",  fontsize=12)
        self.heatmap.ax.set_xlabel('x (pixels)')
        self.heatmap.ax.set_ylabel('y (pixels)')
        self.heatmap.ax.set_zlabel('Average Brillouin shift (GHz)')
        #self.heatmap.fig.colorbar(img, shrink=0.5, aspect=10)

        norm = matplotlib.colors.Normalize(vmin=5, vmax=10)
        
        cb1 = matplotlib.colorbar.ColorbarBase(self.heatmap.ax, cmap=cm.rainbow,
                                norm=norm,
                                orientation='vertical')


        self.heatmap_panel.draw()

        self.setWindowTitle("Brillouin Scan Interface")
        self.move(50,50)
        self.show()
    


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

    app = QApplication(sys.argv)
    gui = App()
    sys.exit(app.exec_())
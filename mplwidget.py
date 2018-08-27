# Imports
from PyQt4 import QtGui

import matplotlib

matplotlib.use('Qt4Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg

# Matplotlib canvas class to create figure
class MplCanvas(FigureCanvasQTAgg):
    def __init__(self):
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        FigureCanvasQTAgg.__init__(self, self.fig)

# Matplotlib widget
class MplWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)   # Inherit from QWidget
        self.canvas = None

    def initialize_canvas(self, figure):
        print "initialized"
        self.canvas = FigureCanvasQTAgg(figure)           # Create canvas object
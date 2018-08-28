# Imports
from PyQt4 import QtGui

import matplotlib

matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg

# Matplotlib widget
class MplWidget(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)   # Inherit from QWidget
        self.canvas = None
        self.vbl = QtGui.QVBoxLayout()

    def initialize_canvas(self, figure):
        print "initialized"
        self.canvas = FigureCanvasQTAgg(figure)           # Create canvas object
        self.vbl.addWidget(self.canvas)
        self.setLayout(self.vbl)
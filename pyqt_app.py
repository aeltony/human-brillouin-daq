import sys
from PyQt4 import QtGui,QtCore


class App(QtGui.QWidget):

	def __init__(self):
		super(App,self).__init__()



		self.mainUI()

	def mainUI(self):


		grid = QtGui.QGridLayout()
		self.setLayout(grid)

		snapshot_btn = QtGui.QPushButton("Picture",self)
		record_btn = QtGui.QPushButton("Record",self)
		record_btn.setCheckable(True)

		grid.addWidget(snapshot_btn,3,0)
		grid.addWidget(record_btn,3,1)


		reference_btn = QtGui.QPushButton("Reference",self)
		reference_btn.setCheckable(True)
		FSR_label = QtGui.QLabel("FSR")
		FSR_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
		FSR_entry = QtGui.QLineEdit()
		SD_label = QtGui.QLabel("SD")
		SD_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
		SD_entry = QtGui.QLineEdit()

		grid.addWidget(reference_btn,3,2)
		grid.addWidget(FSR_label,3,3)
		grid.addWidget(FSR_entry,3,4)
		grid.addWidget(SD_label,3,5)
		grid.addWidget(SD_entry,3,6)


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

		self.setWindowTitle("Brillouin Scan Interface")
		self.move(50,50)
		#self.resize(800,800)
		self.show()











if __name__ == "__main__":
	app = QtGui.QApplication(sys.argv)
	GUI = App()
	sys.exit(app.exec_())

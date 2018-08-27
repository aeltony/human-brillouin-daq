# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'qt_ui.ui'
#
# Created: Mon Aug 27 19:13:13 2018
#      by: PyQt4 UI code generator 4.10
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(2791, 1662)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(754, 13, 1031, 1581))
        self.gridLayoutWidget.setObjectName(_fromUtf8("gridLayoutWidget"))
        self.gridLayout = QtGui.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setMargin(0)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.coord_panel = QtGui.QLabel(self.gridLayoutWidget)
        self.coord_panel.setObjectName(_fromUtf8("coord_panel"))
        self.gridLayout.addWidget(self.coord_panel, 1, 1, 1, 1)
        self.cmos_panel = QtGui.QLabel(self.gridLayoutWidget)
        self.cmos_panel.setObjectName(_fromUtf8("cmos_panel"))
        self.gridLayout.addWidget(self.cmos_panel, 0, 1, 1, 2)
        self.heatmap_panel = MplWidget(self.gridLayoutWidget)
        self.heatmap_panel.setObjectName(_fromUtf8("heatmap_panel"))
        self.gridLayout.addWidget(self.heatmap_panel, 1, 2, 1, 1)
        self.slider = QtGui.QSlider(self.gridLayoutWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.slider.sizePolicy().hasHeightForWidth())
        self.slider.setSizePolicy(sizePolicy)
        self.slider.setMaximum(0)
        self.slider.setOrientation(QtCore.Qt.Horizontal)
        self.slider.setObjectName(_fromUtf8("slider"))
        self.gridLayout.addWidget(self.slider, 2, 2, 1, 1)
        self.gridLayoutWidget_2 = QtGui.QWidget(self.centralwidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(20, 20, 721, 558))
        self.gridLayoutWidget_2.setObjectName(_fromUtf8("gridLayoutWidget_2"))
        self.pupil_detection_panel = QtGui.QGridLayout(self.gridLayoutWidget_2)
        self.pupil_detection_panel.setMargin(0)
        self.pupil_detection_panel.setObjectName(_fromUtf8("pupil_detection_panel"))
        self.radius_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.radius_entry.setObjectName(_fromUtf8("radius_entry"))
        self.pupil_detection_panel.addWidget(self.radius_entry, 8, 1, 1, 1)
        self.minDist_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.minDist_label.setAlignment(QtCore.Qt.AlignCenter)
        self.minDist_label.setObjectName(_fromUtf8("minDist_label"))
        self.pupil_detection_panel.addWidget(self.minDist_label, 4, 0, 1, 1)
        self.param1_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.param1_label.setAlignment(QtCore.Qt.AlignCenter)
        self.param1_label.setObjectName(_fromUtf8("param1_label"))
        self.pupil_detection_panel.addWidget(self.param1_label, 5, 0, 1, 1)
        self.minDist_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.minDist_entry.setObjectName(_fromUtf8("minDist_entry"))
        self.pupil_detection_panel.addWidget(self.minDist_entry, 4, 1, 1, 1)
        self.radius_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.radius_label.setAlignment(QtCore.Qt.AlignCenter)
        self.radius_label.setObjectName(_fromUtf8("radius_label"))
        self.pupil_detection_panel.addWidget(self.radius_label, 8, 0, 1, 1)
        self.range_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.range_label.setAlignment(QtCore.Qt.AlignCenter)
        self.range_label.setObjectName(_fromUtf8("range_label"))
        self.pupil_detection_panel.addWidget(self.range_label, 7, 0, 1, 1)
        self.param2_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.param2_entry.setObjectName(_fromUtf8("param2_entry"))
        self.pupil_detection_panel.addWidget(self.param2_entry, 6, 1, 1, 1)
        self.blur_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.blur_entry.setObjectName(_fromUtf8("blur_entry"))
        self.pupil_detection_panel.addWidget(self.blur_entry, 2, 1, 1, 1)
        self.param1_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.param1_entry.setObjectName(_fromUtf8("param1_entry"))
        self.pupil_detection_panel.addWidget(self.param1_entry, 5, 1, 1, 1)
        self.range_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.range_entry.setObjectName(_fromUtf8("range_entry"))
        self.pupil_detection_panel.addWidget(self.range_entry, 7, 1, 1, 1)
        self.dp_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.dp_label.setAlignment(QtCore.Qt.AlignCenter)
        self.dp_label.setObjectName(_fromUtf8("dp_label"))
        self.pupil_detection_panel.addWidget(self.dp_label, 3, 0, 1, 1)
        self.param2_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.param2_label.setAlignment(QtCore.Qt.AlignCenter)
        self.param2_label.setObjectName(_fromUtf8("param2_label"))
        self.pupil_detection_panel.addWidget(self.param2_label, 6, 0, 1, 1)
        self.blur_label = QtGui.QLabel(self.gridLayoutWidget_2)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.blur_label.sizePolicy().hasHeightForWidth())
        self.blur_label.setSizePolicy(sizePolicy)
        self.blur_label.setAlignment(QtCore.Qt.AlignCenter)
        self.blur_label.setObjectName(_fromUtf8("blur_label"))
        self.pupil_detection_panel.addWidget(self.blur_label, 2, 0, 1, 1)
        self.dp_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.dp_entry.setObjectName(_fromUtf8("dp_entry"))
        self.pupil_detection_panel.addWidget(self.dp_entry, 3, 1, 1, 1)
        self.set_coordinates_btn = QtGui.QPushButton(self.gridLayoutWidget_2)
        self.set_coordinates_btn.setObjectName(_fromUtf8("set_coordinates_btn"))
        self.pupil_detection_panel.addWidget(self.set_coordinates_btn, 10, 3, 1, 1)
        self.croppingSize_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.croppingSize_entry.setObjectName(_fromUtf8("croppingSize_entry"))
        self.pupil_detection_panel.addWidget(self.croppingSize_entry, 9, 1, 1, 1)
        self.croppingSize_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.croppingSize_label.setAlignment(QtCore.Qt.AlignCenter)
        self.croppingSize_label.setObjectName(_fromUtf8("croppingSize_label"))
        self.pupil_detection_panel.addWidget(self.croppingSize_label, 9, 0, 1, 1)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.pupil_detection_panel.addItem(spacerItem, 10, 0, 1, 1)
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.pupil_detection_panel.addItem(spacerItem1, 10, 1, 1, 1)
        spacerItem2 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.pupil_detection_panel.addItem(spacerItem2, 10, 2, 1, 1)
        self.off_btn = QtGui.QPushButton(self.gridLayoutWidget_2)
        self.off_btn.setObjectName(_fromUtf8("off_btn"))
        self.pupil_detection_panel.addWidget(self.off_btn, 13, 3, 1, 1)
        self.set_scan_loc_btn = QtGui.QPushButton(self.gridLayoutWidget_2)
        self.set_scan_loc_btn.setCheckable(True)
        self.set_scan_loc_btn.setObjectName(_fromUtf8("set_scan_loc_btn"))
        self.pupil_detection_panel.addWidget(self.set_scan_loc_btn, 6, 3, 1, 1)
        self.detection_panel_label = QtGui.QLabel(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setKerning(True)
        self.detection_panel_label.setFont(font)
        self.detection_panel_label.setAlignment(QtCore.Qt.AlignCenter)
        self.detection_panel_label.setMargin(10)
        self.detection_panel_label.setObjectName(_fromUtf8("detection_panel_label"))
        self.pupil_detection_panel.addWidget(self.detection_panel_label, 0, 0, 2, 3)
        self.scan_location_x_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.scan_location_x_entry.setObjectName(_fromUtf8("scan_location_x_entry"))
        self.pupil_detection_panel.addWidget(self.scan_location_x_entry, 4, 3, 1, 1)
        self.scan_location_y_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.scan_location_y_entry.setObjectName(_fromUtf8("scan_location_y_entry"))
        self.pupil_detection_panel.addWidget(self.scan_location_y_entry, 5, 3, 1, 1)
        self.scan_location_x_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.scan_location_x_label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.scan_location_x_label.setObjectName(_fromUtf8("scan_location_x_label"))
        self.pupil_detection_panel.addWidget(self.scan_location_x_label, 4, 2, 1, 1)
        self.scan_location_y_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.scan_location_y_label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.scan_location_y_label.setObjectName(_fromUtf8("scan_location_y_label"))
        self.pupil_detection_panel.addWidget(self.scan_location_y_label, 5, 2, 1, 1)
        self.pupil_center_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.pupil_center_label.setAlignment(QtCore.Qt.AlignCenter)
        self.pupil_center_label.setObjectName(_fromUtf8("pupil_center_label"))
        self.pupil_detection_panel.addWidget(self.pupil_center_label, 12, 0, 1, 1)
        self.pupil_radius_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.pupil_radius_label.setAlignment(QtCore.Qt.AlignCenter)
        self.pupil_radius_label.setObjectName(_fromUtf8("pupil_radius_label"))
        self.pupil_detection_panel.addWidget(self.pupil_radius_label, 12, 1, 1, 1)
        self.pupil_center_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.pupil_center_entry.setObjectName(_fromUtf8("pupil_center_entry"))
        self.pupil_detection_panel.addWidget(self.pupil_center_entry, 13, 0, 1, 1)
        self.pupil_radius_entry = QtGui.QLineEdit(self.gridLayoutWidget_2)
        self.pupil_radius_entry.setObjectName(_fromUtf8("pupil_radius_entry"))
        self.pupil_detection_panel.addWidget(self.pupil_radius_entry, 13, 1, 1, 1)
        self.scan_location_label = QtGui.QLabel(self.gridLayoutWidget_2)
        self.scan_location_label.setAlignment(QtCore.Qt.AlignCenter)
        self.scan_location_label.setObjectName(_fromUtf8("scan_location_label"))
        self.pupil_detection_panel.addWidget(self.scan_location_label, 3, 3, 1, 1)
        self.apply_btn = QtGui.QPushButton(self.gridLayoutWidget_2)
        self.apply_btn.setObjectName(_fromUtf8("apply_btn"))
        self.pupil_detection_panel.addWidget(self.apply_btn, 13, 2, 1, 1)
        self.radius_btn = QtGui.QPushButton(self.gridLayoutWidget_2)
        self.radius_btn.setObjectName(_fromUtf8("radius_btn"))
        self.pupil_detection_panel.addWidget(self.radius_btn, 8, 3, 1, 1)
        self.default_btn = QtGui.QPushButton(self.gridLayoutWidget_2)
        self.default_btn.setObjectName(_fromUtf8("default_btn"))
        self.pupil_detection_panel.addWidget(self.default_btn, 9, 3, 1, 1)
        self.gridLayoutWidget_3 = QtGui.QWidget(self.centralwidget)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(20, 600, 551, 281))
        self.gridLayoutWidget_3.setObjectName(_fromUtf8("gridLayoutWidget_3"))
        self.motor_panel = QtGui.QGridLayout(self.gridLayoutWidget_3)
        self.motor_panel.setMargin(0)
        self.motor_panel.setObjectName(_fromUtf8("motor_panel"))
        self.distance_label = QtGui.QLabel(self.gridLayoutWidget_3)
        self.distance_label.setObjectName(_fromUtf8("distance_label"))
        self.motor_panel.addWidget(self.distance_label, 3, 0, 1, 1)
        self.motor_label = QtGui.QLabel(self.gridLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.motor_label.setFont(font)
        self.motor_label.setAlignment(QtCore.Qt.AlignCenter)
        self.motor_label.setObjectName(_fromUtf8("motor_label"))
        self.motor_panel.addWidget(self.motor_label, 0, 0, 1, 4)
        self.acceleration_label = QtGui.QLabel(self.gridLayoutWidget_3)
        self.acceleration_label.setObjectName(_fromUtf8("acceleration_label"))
        self.motor_panel.addWidget(self.acceleration_label, 1, 1, 1, 1)
        self.position_btn = QtGui.QPushButton(self.gridLayoutWidget_3)
        self.position_btn.setObjectName(_fromUtf8("position_btn"))
        self.motor_panel.addWidget(self.position_btn, 8, 1, 1, 1)
        self.location_label = QtGui.QLabel(self.gridLayoutWidget_3)
        self.location_label.setObjectName(_fromUtf8("location_label"))
        self.motor_panel.addWidget(self.location_label, 7, 0, 1, 1)
        self.velocity_label = QtGui.QLabel(self.gridLayoutWidget_3)
        self.velocity_label.setObjectName(_fromUtf8("velocity_label"))
        self.motor_panel.addWidget(self.velocity_label, 1, 0, 1, 1)
        self.acceleration_entry = QtGui.QLineEdit(self.gridLayoutWidget_3)
        self.acceleration_entry.setObjectName(_fromUtf8("acceleration_entry"))
        self.motor_panel.addWidget(self.acceleration_entry, 2, 1, 1, 1)
        self.home_btn = QtGui.QPushButton(self.gridLayoutWidget_3)
        self.home_btn.setObjectName(_fromUtf8("home_btn"))
        self.motor_panel.addWidget(self.home_btn, 8, 2, 1, 1)
        self.forward_btn = QtGui.QPushButton(self.gridLayoutWidget_3)
        self.forward_btn.setObjectName(_fromUtf8("forward_btn"))
        self.motor_panel.addWidget(self.forward_btn, 4, 1, 1, 1)
        self.velocity_entry = QtGui.QLineEdit(self.gridLayoutWidget_3)
        self.velocity_entry.setObjectName(_fromUtf8("velocity_entry"))
        self.motor_panel.addWidget(self.velocity_entry, 2, 0, 1, 1)
        self.distance_entry = QtGui.QLineEdit(self.gridLayoutWidget_3)
        self.distance_entry.setObjectName(_fromUtf8("distance_entry"))
        self.motor_panel.addWidget(self.distance_entry, 4, 0, 1, 1)
        self.backward_btn = QtGui.QPushButton(self.gridLayoutWidget_3)
        self.backward_btn.setObjectName(_fromUtf8("backward_btn"))
        self.motor_panel.addWidget(self.backward_btn, 4, 2, 1, 1)
        self.location_entry = QtGui.QLineEdit(self.gridLayoutWidget_3)
        self.location_entry.setObjectName(_fromUtf8("location_entry"))
        self.motor_panel.addWidget(self.location_entry, 8, 0, 1, 1)
        self.gridLayoutWidget_4 = QtGui.QWidget(self.centralwidget)
        self.gridLayoutWidget_4.setGeometry(QtCore.QRect(20, 900, 551, 81))
        self.gridLayoutWidget_4.setObjectName(_fromUtf8("gridLayoutWidget_4"))
        self.pupil_camera_panel = QtGui.QGridLayout(self.gridLayoutWidget_4)
        self.pupil_camera_panel.setMargin(0)
        self.pupil_camera_panel.setObjectName(_fromUtf8("pupil_camera_panel"))
        self.snapshot_btn = QtGui.QPushButton(self.gridLayoutWidget_4)
        self.snapshot_btn.setObjectName(_fromUtf8("snapshot_btn"))
        self.pupil_camera_panel.addWidget(self.snapshot_btn, 2, 0, 1, 1)
        self.record_btn = QtGui.QPushButton(self.gridLayoutWidget_4)
        self.record_btn.setCheckable(True)
        self.record_btn.setObjectName(_fromUtf8("record_btn"))
        self.pupil_camera_panel.addWidget(self.record_btn, 2, 1, 1, 1)
        self.pupil_camera_label = QtGui.QLabel(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.pupil_camera_label.setFont(font)
        self.pupil_camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.pupil_camera_label.setObjectName(_fromUtf8("pupil_camera_label"))
        self.pupil_camera_panel.addWidget(self.pupil_camera_label, 0, 0, 1, 2)
        self.gridLayoutWidget_5 = QtGui.QWidget(self.centralwidget)
        self.gridLayoutWidget_5.setGeometry(QtCore.QRect(20, 1030, 731, 431))
        self.gridLayoutWidget_5.setObjectName(_fromUtf8("gridLayoutWidget_5"))
        self.data_collection_panel = QtGui.QGridLayout(self.gridLayoutWidget_5)
        self.data_collection_panel.setMargin(0)
        self.data_collection_panel.setObjectName(_fromUtf8("data_collection_panel"))
        self.SD_label = QtGui.QLabel(self.gridLayoutWidget_5)
        self.SD_label.setObjectName(_fromUtf8("SD_label"))
        self.data_collection_panel.addWidget(self.SD_label, 4, 2, 1, 1)
        self.start_pos_label = QtGui.QLabel(self.gridLayoutWidget_5)
        self.start_pos_label.setObjectName(_fromUtf8("start_pos_label"))
        self.data_collection_panel.addWidget(self.start_pos_label, 1, 0, 1, 1)
        self.SD_entry = QtGui.QLineEdit(self.gridLayoutWidget_5)
        self.SD_entry.setObjectName(_fromUtf8("SD_entry"))
        self.data_collection_panel.addWidget(self.SD_entry, 5, 2, 1, 1)
        spacerItem3 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem3, 3, 2, 1, 1)
        self.start_pos = QtGui.QLineEdit(self.gridLayoutWidget_5)
        self.start_pos.setObjectName(_fromUtf8("start_pos"))
        self.data_collection_panel.addWidget(self.start_pos, 2, 0, 1, 1)
        self.num_frames_label = QtGui.QLabel(self.gridLayoutWidget_5)
        self.num_frames_label.setObjectName(_fromUtf8("num_frames_label"))
        self.data_collection_panel.addWidget(self.num_frames_label, 1, 2, 1, 1)
        self.scan_length = QtGui.QLineEdit(self.gridLayoutWidget_5)
        self.scan_length.setObjectName(_fromUtf8("scan_length"))
        self.data_collection_panel.addWidget(self.scan_length, 2, 1, 1, 1)
        self.scan_length_label = QtGui.QLabel(self.gridLayoutWidget_5)
        self.scan_length_label.setObjectName(_fromUtf8("scan_length_label"))
        self.data_collection_panel.addWidget(self.scan_length_label, 1, 1, 1, 1)
        spacerItem4 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem4, 3, 3, 1, 1)
        spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem5, 6, 0, 1, 1)
        self.reference_btn = QtGui.QPushButton(self.gridLayoutWidget_5)
        self.reference_btn.setCheckable(True)
        self.reference_btn.setObjectName(_fromUtf8("reference_btn"))
        self.data_collection_panel.addWidget(self.reference_btn, 5, 0, 1, 1)
        spacerItem6 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem6, 3, 0, 1, 1)
        spacerItem7 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem7, 6, 3, 1, 1)
        spacerItem8 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem8, 6, 1, 1, 1)
        spacerItem9 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem9, 6, 2, 1, 1)
        self.table_label = QtGui.QLabel(self.gridLayoutWidget_5)
        self.table_label.setAlignment(QtCore.Qt.AlignCenter)
        self.table_label.setObjectName(_fromUtf8("table_label"))
        self.data_collection_panel.addWidget(self.table_label, 7, 1, 1, 2)
        spacerItem10 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.data_collection_panel.addItem(spacerItem10, 3, 1, 1, 1)
        self.scan_btn = QtGui.QPushButton(self.gridLayoutWidget_5)
        self.scan_btn.setObjectName(_fromUtf8("scan_btn"))
        self.data_collection_panel.addWidget(self.scan_btn, 2, 3, 1, 1)
        self.FSR_label = QtGui.QLabel(self.gridLayoutWidget_5)
        self.FSR_label.setObjectName(_fromUtf8("FSR_label"))
        self.data_collection_panel.addWidget(self.FSR_label, 4, 1, 1, 1)
        self.num_frames = QtGui.QLineEdit(self.gridLayoutWidget_5)
        self.num_frames.setObjectName(_fromUtf8("num_frames"))
        self.data_collection_panel.addWidget(self.num_frames, 2, 2, 1, 1)
        self.FSR_entry = QtGui.QLineEdit(self.gridLayoutWidget_5)
        self.FSR_entry.setObjectName(_fromUtf8("FSR_entry"))
        self.data_collection_panel.addWidget(self.FSR_entry, 5, 1, 1, 1)
        self.delete_btn = QtGui.QPushButton(self.gridLayoutWidget_5)
        self.delete_btn.setObjectName(_fromUtf8("delete_btn"))
        self.data_collection_panel.addWidget(self.delete_btn, 9, 1, 1, 1)
        self.clear_btn = QtGui.QPushButton(self.gridLayoutWidget_5)
        self.clear_btn.setObjectName(_fromUtf8("clear_btn"))
        self.data_collection_panel.addWidget(self.clear_btn, 9, 2, 1, 1)
        self.scanned_loc_table = QtGui.QTableWidget(self.gridLayoutWidget_5)
        self.scanned_loc_table.setEditTriggers(QtGui.QAbstractItemView.NoEditTriggers)
        self.scanned_loc_table.setRowCount(1)
        self.scanned_loc_table.setColumnCount(6)
        self.scanned_loc_table.setObjectName(_fromUtf8("scanned_loc_table"))
        self.data_collection_panel.addWidget(self.scanned_loc_table, 8, 0, 1, 4)
        self.data_collection_label = QtGui.QLabel(self.gridLayoutWidget_5)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.data_collection_label.setFont(font)
        self.data_collection_label.setAlignment(QtCore.Qt.AlignCenter)
        self.data_collection_label.setObjectName(_fromUtf8("data_collection_label"))
        self.data_collection_panel.addWidget(self.data_collection_label, 0, 0, 1, 3)
        self.splitter = QtGui.QSplitter(self.centralwidget)
        self.splitter.setGeometry(QtCore.QRect(1790, 10, 981, 1451))
        self.splitter.setOrientation(QtCore.Qt.Vertical)
        self.splitter.setObjectName(_fromUtf8("splitter"))
        self.emccd_panel = QtGui.QLabel(self.splitter)
        self.emccd_panel.setObjectName(_fromUtf8("emccd_panel"))
        self.scan_images = QtGui.QListWidget(self.splitter)
        self.scan_images.setObjectName(_fromUtf8("scan_images"))
        self.graph_panel = MplWidget(self.splitter)
        self.graph_panel.setObjectName(_fromUtf8("graph_panel"))
        self.screenshot_btn = QtGui.QPushButton(self.centralwidget)
        self.screenshot_btn.setGeometry(QtCore.QRect(590, 610, 141, 34))
        self.screenshot_btn.setObjectName(_fromUtf8("screenshot_btn"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 2791, 31))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Brillouin Scanning Interface", None))
        self.coord_panel.setText(_translate("MainWindow", "TextLabel", None))
        self.cmos_panel.setText(_translate("MainWindow", "TextLabel", None))
        self.radius_entry.setText(_translate("MainWindow", "180", None))
        self.minDist_label.setText(_translate("MainWindow", "minDist = ", None))
        self.param1_label.setText(_translate("MainWindow", "param1 = ", None))
        self.minDist_entry.setText(_translate("MainWindow", "1000", None))
        self.radius_label.setText(_translate("MainWindow", "radius =", None))
        self.range_label.setText(_translate("MainWindow", "range = ", None))
        self.param2_entry.setText(_translate("MainWindow", "300", None))
        self.blur_entry.setText(_translate("MainWindow", "15", None))
        self.param1_entry.setText(_translate("MainWindow", "1", None))
        self.range_entry.setText(_translate("MainWindow", "15", None))
        self.dp_label.setText(_translate("MainWindow", "dp =", None))
        self.param2_label.setText(_translate("MainWindow", "param2 = ", None))
        self.blur_label.setText(_translate("MainWindow", "medianBlur =", None))
        self.dp_entry.setText(_translate("MainWindow", "3", None))
        self.set_coordinates_btn.setText(_translate("MainWindow", "Set Coordinates", None))
        self.croppingSize_entry.setText(_translate("MainWindow", "300", None))
        self.croppingSize_label.setText(_translate("MainWindow", "croppingSize = ", None))
        self.off_btn.setText(_translate("MainWindow", "Turn Off", None))
        self.set_scan_loc_btn.setText(_translate("MainWindow", "Set Scan Location", None))
        self.detection_panel_label.setText(_translate("MainWindow", "Pupil Detection Panel", None))
        self.scan_location_x_entry.setText(_translate("MainWindow", "598", None))
        self.scan_location_y_entry.setText(_translate("MainWindow", "564", None))
        self.scan_location_x_label.setText(_translate("MainWindow", "x =", None))
        self.scan_location_y_label.setText(_translate("MainWindow", "y =", None))
        self.pupil_center_label.setText(_translate("MainWindow", "Pupil Center", None))
        self.pupil_radius_label.setText(_translate("MainWindow", "Pupil Radius", None))
        self.scan_location_label.setText(_translate("MainWindow", "Scan Location", None))
        self.apply_btn.setText(_translate("MainWindow", "Apply Changes", None))
        self.radius_btn.setText(_translate("MainWindow", "Draw Radius Estimate", None))
        self.default_btn.setText(_translate("MainWindow", "Restore Defaults", None))
        self.distance_label.setText(_translate("MainWindow", "Distance To Move (um)", None))
        self.motor_label.setText(_translate("MainWindow", "Motor Panel", None))
        self.acceleration_label.setText(_translate("MainWindow", "Acceleration (um/s^2)", None))
        self.position_btn.setText(_translate("MainWindow", "Move To Location", None))
        self.location_label.setText(_translate("MainWindow", "Current Location (um)", None))
        self.velocity_label.setText(_translate("MainWindow", "Velocity (um/s)", None))
        self.home_btn.setText(_translate("MainWindow", "Home", None))
        self.forward_btn.setText(_translate("MainWindow", "Forward", None))
        self.distance_entry.setText(_translate("MainWindow", "0", None))
        self.backward_btn.setText(_translate("MainWindow", "Backward", None))
        self.location_entry.setText(_translate("MainWindow", "0", None))
        self.snapshot_btn.setText(_translate("MainWindow", "Take Picture", None))
        self.record_btn.setText(_translate("MainWindow", "Record", None))
        self.pupil_camera_label.setText(_translate("MainWindow", "Pupil Camera Panel", None))
        self.SD_label.setText(_translate("MainWindow", "SD", None))
        self.start_pos_label.setText(_translate("MainWindow", "Start Position (um)", None))
        self.SD_entry.setText(_translate("MainWindow", "0.16", None))
        self.start_pos.setText(_translate("MainWindow", "0", None))
        self.num_frames_label.setText(_translate("MainWindow", "#Frames", None))
        self.scan_length.setText(_translate("MainWindow", "1000", None))
        self.scan_length_label.setText(_translate("MainWindow", "Length (um)", None))
        self.reference_btn.setText(_translate("MainWindow", "Reference", None))
        self.table_label.setText(_translate("MainWindow", "Scanned Point Table", None))
        self.scan_btn.setText(_translate("MainWindow", "Start Scan", None))
        self.FSR_label.setText(_translate("MainWindow", "FSR", None))
        self.num_frames.setText(_translate("MainWindow", "1", None))
        self.FSR_entry.setText(_translate("MainWindow", "16.2566", None))
        self.delete_btn.setText(_translate("MainWindow", "Delete Selected", None))
        self.clear_btn.setText(_translate("MainWindow", "Clear Table", None))
        self.data_collection_label.setText(_translate("MainWindow", "Data Collection Panel", None))
        self.emccd_panel.setText(_translate("MainWindow", "TextLabel", None))
        self.screenshot_btn.setText(_translate("MainWindow", "Take Screenshot", None))

from mplwidget import MplWidget

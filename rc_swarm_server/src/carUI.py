# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'items.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from RcCarClient import DroneConnect
from rc_car_msgs.msg import Diagnostics
from enum import Enum

mode_list = ("ALTCTL","OFFBOARD", "STABILIZED", "AUTO_RTL", "AUTO_LOITER", "POSCTL")

Color = {"RED": 0,"GREEN": 1, "YELLOW": 2}


class Ui(QtWidgets.QWidget):
    def __init__(self, name, rc_car, parent=None):
        super(QtWidgets.QWidget,self).__init__(parent)

        self.car_client = rc_car
        self.name = name
        self.centralwidget = QtWidgets.QWidget()
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(0, 70, 621, 81))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.ItemLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.ItemLayout.setObjectName("ItemLayout")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.GpsCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.GpsCheckBox.setMaximumSize(QtCore.QSize(52, 16777215))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(9)
        self.GpsCheckBox.setFont(font)
        self.GpsCheckBox.setObjectName("GpsCheckBox")
        self.horizontalLayout_2.addWidget(self.GpsCheckBox)
        self.HomeCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.HomeCheckBox.setMinimumSize(QtCore.QSize(0, 0))
        self.HomeCheckBox.setMaximumSize(QtCore.QSize(64, 16777215))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(9)
        self.HomeCheckBox.setFont(font)
        self.HomeCheckBox.setObjectName("HomeCheckBox")
        self.horizontalLayout_2.addWidget(self.HomeCheckBox)
        self.horizontalLayout_4.addLayout(self.horizontalLayout_2)
        self.ItemLayout.addLayout(self.horizontalLayout_4, 0, 3, 1, 1)
        self.SelectCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.SelectCheckBox.setMinimumSize(QtCore.QSize(24, 32))
        self.SelectCheckBox.setMaximumSize(QtCore.QSize(24, 24))
        self.SelectCheckBox.setMouseTracking(True)
        self.SelectCheckBox.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.SelectCheckBox.setText("")
        self.SelectCheckBox.setIconSize(QtCore.QSize(32, 32))
        self.SelectCheckBox.setCheckable(True)
        self.SelectCheckBox.setObjectName("SelectCheckBox")
        self.ItemLayout.addWidget(self.SelectCheckBox, 1, 0, 1, 1)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.ipEdit = QtWidgets.QTextEdit(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ipEdit.sizePolicy().hasHeightForWidth())
        self.ipEdit.setSizePolicy(sizePolicy)
        self.ipEdit.setMinimumSize(QtCore.QSize(164, 0))
        self.ipEdit.setMaximumSize(QtCore.QSize(164, 32))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.ipEdit.setFont(font)
        self.ipEdit.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.IBeamCursor))
        self.ipEdit.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.ipEdit.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.ipEdit.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.ipEdit.setObjectName("ipEdit")
        self.horizontalLayout_3.addWidget(self.ipEdit)
        self.portEdit = QtWidgets.QTextEdit(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.portEdit.sizePolicy().hasHeightForWidth())
        self.portEdit.setSizePolicy(sizePolicy)
        self.portEdit.setMaximumSize(QtCore.QSize(56, 32))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(10)
        self.portEdit.setFont(font)
        self.portEdit.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.IBeamCursor))
        self.portEdit.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.portEdit.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.portEdit.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.portEdit.setObjectName("portEdit")
        self.horizontalLayout_3.addWidget(self.portEdit)
        self.ItemLayout.addLayout(self.horizontalLayout_3, 1, 1, 1, 1)
        self.ModeComboBox = QtWidgets.QComboBox(self.gridLayoutWidget)
        self.ModeComboBox.setMaximumSize(QtCore.QSize(128, 16777215))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        self.ModeComboBox.setFont(font)
        self.ModeComboBox.setObjectName("ModeComboBox")
        self.ItemLayout.addWidget(self.ModeComboBox, 1, 3, 1, 1)
        self.ArmButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.ArmButton.setMaximumSize(QtCore.QSize(84, 16777215))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.ArmButton.setFont(font)
        self.ArmButton.setCursor(QtGui.QCursor(QtCore.Qt.OpenHandCursor))
        self.ArmButton.setObjectName("ArmButton")
        self.ItemLayout.addWidget(self.ArmButton, 1, 5, 1, 1)
        self.ConnectButton = QtWidgets.QPushButton(self.gridLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ConnectButton.sizePolicy().hasHeightForWidth())
        self.ConnectButton.setSizePolicy(sizePolicy)
        self.ConnectButton.setMaximumSize(QtCore.QSize(96, 32))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(10)
        self.ConnectButton.setFont(font)
        self.ConnectButton.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.ConnectButton.setObjectName("ConnectButton")
        self.ItemLayout.addWidget(self.ConnectButton, 1, 2, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(10, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.ItemLayout.addItem(spacerItem, 1, 4, 1, 1)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.NameLabel_2 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.NameLabel_2.setMaximumSize(QtCore.QSize(96, 32))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.NameLabel_2.setFont(font)
        self.NameLabel_2.setObjectName("NameLabel_2")
        self.horizontalLayout.addWidget(self.NameLabel_2)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.BatteryTextLabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.BatteryTextLabel.setMaximumSize(QtCore.QSize(34, 32))
        self.BatteryTextLabel.setObjectName("BatteryTextLabel")
        self.horizontalLayout.addWidget(self.BatteryTextLabel)
        self.Batterylabel = QtWidgets.QLabel(self.gridLayoutWidget)
        self.Batterylabel.setMaximumSize(QtCore.QSize(56, 16777215))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setBold(True)
        font.setWeight(75)
        self.Batterylabel.setFont(font)
        self.Batterylabel.setObjectName("Batterylabel")
        self.horizontalLayout.addWidget(self.Batterylabel)
        self.ItemLayout.addLayout(self.horizontalLayout, 0, 1, 1, 1)
        self.OriginCheckBox = QtWidgets.QCheckBox(self.gridLayoutWidget)
        self.OriginCheckBox.setMinimumSize(QtCore.QSize(64, 0))
        self.OriginCheckBox.setMaximumSize(QtCore.QSize(64, 16777215))
        font = QtGui.QFont()
        font.setFamily("Roboto")
        font.setPointSize(9)
        self.OriginCheckBox.setFont(font)
        self.OriginCheckBox.setObjectName("OriginCheckBox")
        self.ItemLayout.addWidget(self.OriginCheckBox, 0, 2, 1, 1)


        self.widget = QtWidgets.QWidget(self.gridLayoutWidget)

        self._changeColorStateOfItem(Color["RED"])
        self.widget.setAutoFillBackground(True)
        self.widget.setObjectName("widget")
        self.ItemLayout.addWidget(self.widget, 0, 0, 1, 1)


        QtCore.QMetaObject.connectSlotsByName(self)

        self.setLayout(self.ItemLayout)

        self.set_text()
        self.initSlots()

    def set_text(self):
        _translate = QtCore.QCoreApplication.translate
        self.GpsCheckBox.setText(_translate("MainWindow", "gps"))
        self.HomeCheckBox.setText(_translate("MainWindow", "home"))
        self.ArmButton.setText(_translate("MainWindow", "Arm"))
        self.ConnectButton.setText(_translate("MainWindow", "Connect"))
        self.NameLabel_2.setText(_translate("MainWindow", self.name))
        self.BatteryTextLabel.setText(_translate("MainWindow", "bat:"))
        self.Batterylabel.setText(_translate("MainWindow", "100%"))
        self.OriginCheckBox.setText(_translate("MainWindow", "origin"))

        self.ipEdit.setText(str(self.car_client.ws._ip))
        self.portEdit.setText(str(self.car_client.ws._port))

    def initSlots(self):
        self.portEdit.setText(str(self.car_client.ws._port))
        self.ipEdit.setText(str(self.car_client.ws._ip))

        self.ConnectButton.clicked.connect(self.connect)
        self.ArmButton.clicked.connect(self.arm)
        self.ipEdit.textChanged.connect(self.setIp)
        self.portEdit.textChanged.connect(self.setPort)

        self.ModeComboBox.addItems(mode_list)
        self.ModeComboBox.activated[str].connect(self.setMode)
        self.car_client.diag_signals.connect(self._getDiag)
        self.car_client.ws.connect_signal.connect(self._changeConnect)

    def _getDiag(self):
        self._changeGuiElem(self.car_client.diagnostics)

    def _changeGuiElem(self, diag_data):
        """
        :type diag_data: Diagnostics
        :return:
        """
        # combo box
        self.changeComboBox(diag_data.mode)
        self.Batterylabel.setText('{0:.0%}'.format(diag_data.battery))

        # arm button
        armText = "Arm" if not diag_data.armed else "disarm"
        self.ArmButton.setText(armText)
        # status gps
        if diag_data.gps_send != self.GpsCheckBox.isChecked():
            self.GpsCheckBox.click()
        if diag_data.init_home != self.HomeCheckBox.isChecked():
            self.HomeCheckBox.click()
        if diag_data.init_origin != self.OriginCheckBox.isChecked():
            self.OriginCheckBox.click()

        if diag_data.init_home and diag_data.gps_send and diag_data.init_origin:
            self._changeColorStateOfItem(Color["GREEN"])
        else:
            self._changeColorStateOfItem(Color["YELLOW"])

    def _changeConnect(self, state):

        if state == True:
            self._changeColorStateOfItem(Color["YELLOW"])
            self.ConnectButton.setText("Disconnect")
        if state == False:
            self._changeColorStateOfItem(Color["RED"])
            self.ConnectButton.setText("Connect")

    def _changeColorStateOfItem(self, _color):

        if _color == Color["RED"]:
            colorBg = QtGui.QColor(255, 0, 0)
        elif _color == Color["GREEN"]:
            colorBg = QtGui.QColor(0, 255, 0)
        elif _color == Color["YELLOW"]:
            colorBg = QtGui.QColor(255, 255, 0)
        else:
            print("not set color, breack")
            return
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Base, colorBg)
        self.widget.setPalette(palette)

    def setMode(self, mode):
        self.car_client.set_mode(mode)

    def changeComboBox(self, mode):
        self.ModeComboBox.setCurrentText(mode)

    def setIp(self):
        data = self.ipEdit.toPlainText()
        self.car_client.ws.setIp(data)

    def setPort(self):
        data = self.portEdit.toPlainText()
        self.car_client.ws.setPort(data)

    def connect(self):
        """
        Connect to server
        :type: self.car_client
        """
        if not self.car_client.is_active():
            print("not connect")
            self.car_client.ws.connect()
            return

        if self.car_client.is_active():
            self.car_client.ws.disconnect()
            return

    def arm(self):

        if not self.car_client.is_active():
            print("not connect return")
            return
        if self.car_client.diagnostics.armed:
            self.car_client.disarm()
        else:
            self.car_client.arm()

    def __del__(self):
        self.car_client.__del__()
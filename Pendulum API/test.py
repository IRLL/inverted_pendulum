# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pendulum.ui'
#
# Created: Thu Feb 26 09:03:14 2015
#      by: PyQt4 UI code generator 4.11.3
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
		MainWindow.resize(220, 215)
		sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
		sizePolicy.setHorizontalStretch(0)
		sizePolicy.setVerticalStretch(0)
		sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
		MainWindow.setSizePolicy(sizePolicy)
		MainWindow.setMinimumSize(QtCore.QSize(220, 215))
		MainWindow.setMaximumSize(QtCore.QSize(220, 215))
		MainWindow.setAutoFillBackground(False)
		MainWindow.setStyleSheet(_fromUtf8("background-color: rgb(0,0,0); color: rgb(255,255,255);"))
		MainWindow.setTabShape(QtGui.QTabWidget.Rounded)
		self.centralwidget = QtGui.QWidget(MainWindow)
		self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
		self.gridLayoutWidget = QtGui.QWidget(self.centralwidget)
		self.gridLayoutWidget.setGeometry(QtCore.QRect(10, 10, 201, 163))
		self.gridLayoutWidget.setObjectName(_fromUtf8("gridLayoutWidget"))
		self.gridLayout = QtGui.QGridLayout(self.gridLayoutWidget)
		self.gridLayout.setMargin(0)
		self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
		self.voltage = QtGui.QLCDNumber(self.gridLayoutWidget)
		self.voltage.setSmallDecimalPoint(True)
		self.voltage.setObjectName(_fromUtf8("voltage"))
		self.gridLayout.addWidget(self.voltage, 2, 1, 1, 1)
		self.label5 = QtGui.QLabel(self.gridLayoutWidget)
		font = QtGui.QFont()
		font.setPointSize(10)
		font.setBold(True)
		font.setWeight(75)
		self.label5.setFont(font)
		self.label5.setAlignment(QtCore.Qt.AlignCenter)
		self.label5.setObjectName(_fromUtf8("label5"))
		self.gridLayout.addWidget(self.label5, 4, 0, 1, 1)
		self.label1 = QtGui.QLabel(self.gridLayoutWidget)
		font = QtGui.QFont()
		font.setPointSize(10)
		font.setBold(True)
		font.setWeight(75)
		self.label1.setFont(font)
		self.label1.setAlignment(QtCore.Qt.AlignCenter)
		self.label1.setObjectName(_fromUtf8("label1"))
		self.gridLayout.addWidget(self.label1, 0, 0, 1, 1)
		self.armAngle = QtGui.QLCDNumber(6, self.gridLayoutWidget)
		self.armAngle.setSmallDecimalPoint(True)
		self.armAngle.setObjectName(_fromUtf8("armAngle"))
		self.gridLayout.addWidget(self.armAngle, 4, 1, 1, 1)
		self.label4 = QtGui.QLabel(self.gridLayoutWidget)
		font = QtGui.QFont()
		font.setPointSize(10)
		font.setBold(True)
		font.setWeight(75)
		self.label4.setFont(font)
		self.label4.setAlignment(QtCore.Qt.AlignCenter)
		self.label4.setObjectName(_fromUtf8("label4"))
		self.gridLayout.addWidget(self.label4, 3, 0, 1, 1)
		self.speed = QtGui.QLCDNumber(4, self.gridLayoutWidget)
		self.speed.setObjectName(_fromUtf8("speed"))
		self.gridLayout.addWidget(self.speed, 3, 1, 1, 1)
		self.label_2 = QtGui.QLabel(self.gridLayoutWidget)
		font = QtGui.QFont()
		font.setPointSize(10)
		font.setBold(True)
		font.setWeight(75)
		self.label_2.setFont(font)
		self.label_2.setAlignment(QtCore.Qt.AlignCenter)
		self.label_2.setObjectName(_fromUtf8("label_2"))
		self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
		self.eCode = QtGui.QLCDNumber(self.gridLayoutWidget)
		self.eCode.setHexMode()
		self.eCode.setObjectName(_fromUtf8("eCode"))
		self.gridLayout.addWidget(self.eCode, 0, 1, 1, 1)
		self.label3 = QtGui.QLabel(self.gridLayoutWidget)
		font = QtGui.QFont()
		font.setPointSize(10)
		font.setBold(True)
		font.setWeight(75)
		self.label3.setFont(font)
		self.label3.setAlignment(QtCore.Qt.AlignCenter)
		self.label3.setObjectName(_fromUtf8("label3"))
		self.gridLayout.addWidget(self.label3, 2, 0, 1, 1)
		self.temp = QtGui.QLCDNumber(self.gridLayoutWidget)
		self.temp.setObjectName(_fromUtf8("temp"))
		self.gridLayout.addWidget(self.temp, 1, 1, 1, 1)
		self.label6 = QtGui.QLabel(self.gridLayoutWidget)
		font = QtGui.QFont()
		font.setPointSize(10)
		font.setBold(True)
		font.setWeight(75)
		self.label6.setFont(font)
		self.label6.setAlignment(QtCore.Qt.AlignCenter)
		self.label6.setObjectName(_fromUtf8("label6"))
		self.gridLayout.addWidget(self.label6, 5, 0, 1, 1)
		self.position = QtGui.QLCDNumber(self.gridLayoutWidget)
		self.position.setObjectName(_fromUtf8("position"))
		self.gridLayout.addWidget(self.position, 5, 1, 1, 1)
		MainWindow.setCentralWidget(self.centralwidget)
		self.menubar = QtGui.QMenuBar(MainWindow)
		self.menubar.setGeometry(QtCore.QRect(0, 0, 220, 21))
		self.menubar.setObjectName(_fromUtf8("menubar"))
		MainWindow.setMenuBar(self.menubar)
		self.statusbar = QtGui.QStatusBar(MainWindow)
		self.statusbar.setObjectName(_fromUtf8("statusbar"))
		MainWindow.setStatusBar(self.statusbar)

		self.retranslateUi(MainWindow)
		QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Pendulum", None))
        self.label5.setText(_translate("MainWindow", "Arm Angle:", None))
        self.label1.setText(_translate("MainWindow", "Error Code:", None))
        self.label4.setText(_translate("MainWindow", "Speed:", None))
        self.label_2.setText(_translate("MainWindow", "Temperature:", None))
        self.label3.setText(_translate("MainWindow", "Voltage:", None))
        self.label6.setText(_translate("MainWindow", "Cart Position:", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())


# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'QTRehaZenterGUI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
import os,sys,inspect
# include parent "src" directory to sys.path, otherwise import won't work
# (source: http://stackoverflow.com/questions/714063/importing-modules-from-parent-folder)
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
import Exercise1

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

# The new Stream Object which replaces the default stream associated with sys.stdout
# This object just puts data in a queue!
#class WriteStream(object):
#    def __init__(self, queue):
#        self.queue = queue
#
#    def write(self, text):
#        self.queue.put(text)


# solution below taken from: http://stackoverflow.com/questions/8356336/how-to-capture-output-of-pythons-interpreter-and-show-in-a-text-widget
class EmittingStream(QtCore.QObject):
    textWritten = QtCore.pyqtSignal(str)

    def write(self, text):
        self.textWritten.emit(str(text))

class Ui_QTRehaZenter(object):
    def setupUi(self, QTRehaZenter):
        QTRehaZenter.setObjectName(_fromUtf8("QTRehaZenter"))
        QTRehaZenter.resize(957, 1089)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(QTRehaZenter.sizePolicy().hasHeightForWidth())
        QTRehaZenter.setSizePolicy(sizePolicy)
        self.centralwidget = QtGui.QWidget(QTRehaZenter)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.line = QtGui.QFrame(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line.sizePolicy().hasHeightForWidth())
        self.line.setSizePolicy(sizePolicy)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.gridLayout.addWidget(self.line, 7, 0, 1, 2)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setSizeConstraint(QtGui.QLayout.SetDefaultConstraint)
        self.horizontalLayout.setContentsMargins(-1, 0, -1, -1)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.btnBegin = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnBegin.sizePolicy().hasHeightForWidth())
        self.btnBegin.setSizePolicy(sizePolicy)
        self.btnBegin.setMinimumSize(QtCore.QSize(200, 120))
        self.btnBegin.setObjectName(_fromUtf8("btnBegin"))
        self.horizontalLayout.addWidget(self.btnBegin)
        self.btnStop = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnStop.sizePolicy().hasHeightForWidth())
        self.btnStop.setSizePolicy(sizePolicy)
        self.btnStop.setMinimumSize(QtCore.QSize(200, 120))
        self.btnStop.setCheckable(False)
        self.btnStop.setObjectName(_fromUtf8("btnStop"))
	self.btnStop.setEnabled(False)
        self.horizontalLayout.addWidget(self.btnStop)
        self.gridLayout.addLayout(self.horizontalLayout, 6, 0, 1, 2)
        self.lblROSConsole = QtGui.QLabel(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lblROSConsole.sizePolicy().hasHeightForWidth())
        self.lblROSConsole.setSizePolicy(sizePolicy)
        self.lblROSConsole.setObjectName(_fromUtf8("lblROSConsole"))
        self.gridLayout.addWidget(self.lblROSConsole, 8, 0, 1, 1)
        self.btnExercise4 = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnExercise4.sizePolicy().hasHeightForWidth())
        self.btnExercise4.setSizePolicy(sizePolicy)
        self.btnExercise4.setMinimumSize(QtCore.QSize(0, 200))
        self.btnExercise4.setObjectName(_fromUtf8("btnExercise4"))
        self.gridLayout.addWidget(self.btnExercise4, 3, 1, 1, 1)
        self.btnExercise2 = QtGui.QPushButton(self.centralwidget)
        self.btnExercise2.setEnabled(True)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnExercise2.sizePolicy().hasHeightForWidth())
        self.btnExercise2.setSizePolicy(sizePolicy)
        self.btnExercise2.setMinimumSize(QtCore.QSize(0, 200))
        self.btnExercise2.setObjectName(_fromUtf8("btnExercise2"))
        self.gridLayout.addWidget(self.btnExercise2, 2, 1, 1, 1)
        self.line_2 = QtGui.QFrame(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line_2.sizePolicy().hasHeightForWidth())
        self.line_2.setSizePolicy(sizePolicy)
        self.line_2.setFrameShape(QtGui.QFrame.HLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.gridLayout.addWidget(self.line_2, 5, 0, 1, 2)
        self.btnExercise3 = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnExercise3.sizePolicy().hasHeightForWidth())
        self.btnExercise3.setSizePolicy(sizePolicy)
        self.btnExercise3.setMinimumSize(QtCore.QSize(0, 200))
        self.btnExercise3.setObjectName(_fromUtf8("btnExercise3"))
        self.gridLayout.addWidget(self.btnExercise3, 3, 0, 1, 1)
        self.btnExercise1 = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnExercise1.sizePolicy().hasHeightForWidth())
        self.btnExercise1.setSizePolicy(sizePolicy)
        self.btnExercise1.setMinimumSize(QtCore.QSize(0, 200))
        self.btnExercise1.setObjectName(_fromUtf8("btnExercise1"))
        self.gridLayout.addWidget(self.btnExercise1, 2, 0, 1, 1)
        self.txtViewROSConsole = QtGui.QPlainTextEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.txtViewROSConsole.sizePolicy().hasHeightForWidth())
        self.txtViewROSConsole.setSizePolicy(sizePolicy)
        self.txtViewROSConsole.setMinimumSize(QtCore.QSize(751, 400))
        self.txtViewROSConsole.setObjectName(_fromUtf8("txtViewROSConsole"))
        self.gridLayout.addWidget(self.txtViewROSConsole, 9, 0, 1, 2)
        QTRehaZenter.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(QTRehaZenter)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 957, 35))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuEdit = QtGui.QMenu(self.menubar)
        self.menuEdit.setObjectName(_fromUtf8("menuEdit"))
        self.menuHelp = QtGui.QMenu(self.menubar)
        self.menuHelp.setObjectName(_fromUtf8("menuHelp"))
        QTRehaZenter.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(QTRehaZenter)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        QTRehaZenter.setStatusBar(self.statusbar)
        self.toolBar = QtGui.QToolBar(QTRehaZenter)
        self.toolBar.setObjectName(_fromUtf8("toolBar"))
        QTRehaZenter.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.toolBar_2 = QtGui.QToolBar(QTRehaZenter)
        self.toolBar_2.setObjectName(_fromUtf8("toolBar_2"))
        QTRehaZenter.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar_2)
        self.actionSettings = QtGui.QAction(QTRehaZenter)
        self.actionSettings.setObjectName(_fromUtf8("actionSettings"))
        self.actionAbout = QtGui.QAction(QTRehaZenter)
        self.actionAbout.setObjectName(_fromUtf8("actionAbout"))
        self.actionQuit = QtGui.QAction(QTRehaZenter)
        self.actionQuit.setObjectName(_fromUtf8("actionQuit"))
        self.menuEdit.addAction(self.actionSettings)
        self.menuEdit.addSeparator()
        self.menuEdit.addAction(self.actionQuit)
        self.menuHelp.addAction(self.actionAbout)
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi(QTRehaZenter)
        QtCore.QMetaObject.connectSlotsByName(QTRehaZenter)

	# connect functions to buttons
	self.btnExercise3.clicked.connect(self.btnExercise3Clicked)
	self.btnExercise4.clicked.connect(self.btnExercise4Clicked)
	self.btnExercise2.clicked.connect(self.btnExercise2Clicked)
	self.btnExercise1.clicked.connect(self.btnExercise1Clicked)
	self.btnBegin.clicked.connect(self.btnBeginClicked)
	self.btnStop.clicked.connect(self.btnStopClicked)

	#stdout = WriteStream(self)
	#stdout.outputWritten.connect(self.handleROSOutput)
	#sys.stdout = EmittingStream(textWritten=self.normalOutputWritten)

    #def normalOutputWritten(self, text):
    #	self.txtViewROSConsole.appendPlainText(text)

    def retranslateUi(self, QTRehaZenter):
        QTRehaZenter.setWindowTitle(_translate("QTRehaZenter", "MainWindow", None))
        self.btnBegin.setText(_translate("QTRehaZenter", "Begin!", None))
        self.btnStop.setText(_translate("QTRehaZenter", "Stop", None))
        self.lblROSConsole.setText(_translate("QTRehaZenter", "ROS Console output:", None))
        self.btnExercise4.setText(_translate("QTRehaZenter", "Exercise 4", None))
        self.btnExercise2.setText(_translate("QTRehaZenter", "Exercise 2", None))
        self.btnExercise3.setText(_translate("QTRehaZenter", "Exercise 3", None))
        self.btnExercise1.setText(_translate("QTRehaZenter", "Exercise 1", None))
        self.menuEdit.setTitle(_translate("QTRehaZenter", "File", None))
        self.menuHelp.setTitle(_translate("QTRehaZenter", "Help", None))
        self.toolBar.setWindowTitle(_translate("QTRehaZenter", "toolBar", None))
        self.toolBar_2.setWindowTitle(_translate("QTRehaZenter", "toolBar_2", None))
        self.actionSettings.setText(_translate("QTRehaZenter", "Preferences", None))
        self.actionAbout.setText(_translate("QTRehaZenter", "About...", None))
        self.actionQuit.setText(_translate("QTRehaZenter", "Quit", None))

    def __del__(self):
    	# Restore sys.stdout
    	sys.stdout = sys.__stdout__

    # *******************************************************************************************
    # *************************  connector functions for the UI buttons  ************************
    # *******************************************************************************************
    selectedButton = 0
    def btnExercise1Clicked(self):
	# enable all other buttons except the one for exercise 1
	self.btnExercise1.setEnabled(False)
	self.btnExercise2.setEnabled(True)
	self.btnExercise3.setEnabled(True)
	self.btnExercise4.setEnabled(True)
	self.selectedButton = 1
	self.txtViewROSConsole.appendPlainText("Exercise 1 selected.")
	
    def btnExercise2Clicked(self):
	# enable all other buttons except the one for exercise 2
	self.btnExercise1.setEnabled(True)
	self.btnExercise2.setEnabled(False)
	self.btnExercise3.setEnabled(True)
	self.btnExercise4.setEnabled(True)
	self.selectedButton = 2
	self.txtViewROSConsole.appendPlainText("Exercise 2 selected.")

    def btnExercise3Clicked(self):
	# enable all other buttons except the one for exercise 3
	self.btnExercise1.setEnabled(True)
	self.btnExercise2.setEnabled(True)
	self.btnExercise3.setEnabled(False)
	self.btnExercise4.setEnabled(True)
	self.selectedButton = 3
	self.txtViewROSConsole.appendPlainText("Exercise 3 selected.")

    def btnExercise4Clicked(self):
	# enable all other buttons except the one for exercise 4
	self.btnExercise1.setEnabled(True)
	self.btnExercise2.setEnabled(True)
	self.btnExercise3.setEnabled(True)
	self.btnExercise4.setEnabled(False)
	self.selectedButton = 4
	self.txtViewROSConsole.appendPlainText("Exercise 4 selected.")

    def btnBeginClicked(self):
	# disable all other buttons while the chosen exercise is running
	self.btnExercise1.setEnabled(False)
	self.btnExercise2.setEnabled(False)
	self.btnExercise3.setEnabled(False)
	self.btnExercise4.setEnabled(False)
	self.btnBegin.setEnabled(False)
	self.btnStop.setEnabled(True)
	self.txtViewROSConsole.appendPlainText("Exercise " + str(self.selectedButton) + ": STARTED!")
	if self.selectedButton == 1:
		argv = ["--width", "640", "--height", "480", "--color", "yellow", "--calibrate"]
		ex1game = Exercise1.Exercise1Thread(argv)
		ex1game.start()

	# TODO: run ROS launch file and print ROS log stuff on GUI

    def btnStopClicked(self):
	# enable all other buttons again
	self.btnExercise1.setEnabled(True)
	self.btnExercise2.setEnabled(True)
	self.btnExercise3.setEnabled(True)
	self.btnExercise4.setEnabled(True)
	self.btnBegin.setEnabled(True)
	self.btnStop.setEnabled(False)
	selectedButton = 0
	self.txtViewROSConsole.appendPlainText("Exercise " + str(self.selectedButton) + ": STOPPED!")

    def handleROSOutput(self, ):
	self.txtViewROSConsole.appendPlainText()

    # *******************************************************************************************

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    QTRehaZenter = QtGui.QMainWindow()
    ui = Ui_QTRehaZenter()
    ui.setupUi(QTRehaZenter)
    QTRehaZenter.show()
    sys.exit(app.exec_())


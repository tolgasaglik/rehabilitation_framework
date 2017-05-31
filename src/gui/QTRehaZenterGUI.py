# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'self.UI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import QThread
from subprocess import Popen,PIPE
from PyQt4.QtGui import QWidget
import os,sys,inspect
from threading import Thread
from time import sleep
import roslaunch
import rospy
# include parent "src" directory to sys.path, otherwise import won't work
# (source: http://stackoverflow.com/questions/714063/importing-modules-from-parent-folder)
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
import Exercises
from Exercises import Color,Limb,RotationType,MotionType
import QTRehaZenterGUI_Preferences

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


class QTRehaZenterGUI(QtGui.QMainWindow):
    def __init__(self):
	QtGui.QWidget.__init__(self)
	self.initUi()

	# initialize exercise parameters
	self.exercise_width = 640
        self.exercise_height = 480
        self.exercise_color = "yellow"
        self.exercise_number_of_repetitions = 10
        self.exercise_time_limit = 0

	# initialize preferences widget
	self.preferences = QTRehaZenterGUI_Preferences.UIPreferencesWidget(self)

    # encourager unit takes care of ROS communications and counts repetitions
    def initUi(self):
        self.setObjectName(_fromUtf8("QTRehaZenter"))
        self.resize(957, 1089)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(sizePolicy)
        self.centralwidget = QtGui.QWidget(self)
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
	self.btnBegin.setEnabled(False)
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
        self.btnExternalRotationExercise = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnExternalRotationExercise.sizePolicy().hasHeightForWidth())
        self.btnExternalRotationExercise.setSizePolicy(sizePolicy)
        self.btnExternalRotationExercise.setMinimumSize(QtCore.QSize(0, 200))
        self.btnExternalRotationExercise.setObjectName(_fromUtf8("btnExternalRotationExercise"))
        self.gridLayout.addWidget(self.btnExternalRotationExercise, 3, 1, 1, 1)
        self.btnAbductionMotionExercise = QtGui.QPushButton(self.centralwidget)
        self.btnAbductionMotionExercise.setEnabled(True)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnAbductionMotionExercise.sizePolicy().hasHeightForWidth())
        self.btnAbductionMotionExercise.setSizePolicy(sizePolicy)
        self.btnAbductionMotionExercise.setMinimumSize(QtCore.QSize(0, 200))
        self.btnAbductionMotionExercise.setObjectName(_fromUtf8("btnAbductionMotionExercise"))
        self.gridLayout.addWidget(self.btnAbductionMotionExercise, 2, 1, 1, 1)
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
        self.btnInternalRotationExercise = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnInternalRotationExercise.sizePolicy().hasHeightForWidth())
        self.btnInternalRotationExercise.setSizePolicy(sizePolicy)
        self.btnInternalRotationExercise.setMinimumSize(QtCore.QSize(0, 200))
        self.btnInternalRotationExercise.setObjectName(_fromUtf8("btnInternalRotationExercise"))
        self.gridLayout.addWidget(self.btnInternalRotationExercise, 3, 0, 1, 1)
        self.btnFlexionMotionExercise = QtGui.QPushButton(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnFlexionMotionExercise.sizePolicy().hasHeightForWidth())
        self.btnFlexionMotionExercise.setSizePolicy(sizePolicy)
        self.btnFlexionMotionExercise.setMinimumSize(QtCore.QSize(0, 200))
        self.btnFlexionMotionExercise.setObjectName(_fromUtf8("btnFlexionMotionExercise"))
        self.gridLayout.addWidget(self.btnFlexionMotionExercise, 2, 0, 1, 1)
        self.txtViewROSConsole = QtGui.QPlainTextEdit(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.txtViewROSConsole.sizePolicy().hasHeightForWidth())
        self.txtViewROSConsole.setSizePolicy(sizePolicy)
        self.txtViewROSConsole.setMinimumSize(QtCore.QSize(751, 400))
        self.txtViewROSConsole.setObjectName(_fromUtf8("txtViewROSConsole"))
        self.gridLayout.addWidget(self.txtViewROSConsole, 9, 0, 1, 2)
        self.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(self)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 957, 35))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuEdit = QtGui.QMenu(self.menubar)
        self.menuEdit.setObjectName(_fromUtf8("menuEdit"))
        self.menuHelp = QtGui.QMenu(self.menubar)
        self.menuHelp.setObjectName(_fromUtf8("menuHelp"))
        self.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(self)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        self.setStatusBar(self.statusbar)
        self.toolBar = QtGui.QToolBar(self)
        self.toolBar.setObjectName(_fromUtf8("toolBar"))
        self.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.toolBar_2 = QtGui.QToolBar(self)
        self.toolBar_2.setObjectName(_fromUtf8("toolBar_2"))
        self.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar_2)
        self.actionPreferences = QtGui.QAction(self)
        self.actionPreferences.setObjectName(_fromUtf8("actionPreferences"))
        self.actionAbout = QtGui.QAction(self)
        self.actionAbout.setObjectName(_fromUtf8("actionAbout"))
        self.actionQuit = QtGui.QAction(self)
        self.actionQuit.setObjectName(_fromUtf8("actionQuit"))
        self.menuEdit.addAction(self.actionPreferences)
        self.menuEdit.addSeparator()
        self.menuEdit.addAction(self.actionQuit)
        self.menuHelp.addAction(self.actionAbout)
        self.menubar.addAction(self.menuEdit.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self)

	# connect functions to buttons
	self.btnInternalRotationExercise.clicked.connect(self.btnInternalRotationExerciseClicked)
	self.btnExternalRotationExercise.clicked.connect(self.btnExternalRotationExerciseClicked)
	self.btnAbductionMotionExercise.clicked.connect(self.btnAbductionMotionExerciseClicked)
	self.btnFlexionMotionExercise.clicked.connect(self.btnFlexionMotionExerciseClicked)
	self.btnBegin.clicked.connect(self.btnBeginClicked)
	self.btnStop.clicked.connect(self.btnStopClicked)
	self.actionQuit.triggered.connect(self.closeEvent)
	self.actionPreferences.triggered.connect(self.openPreferences)


    def retranslateUi(self):
        self.setWindowTitle(_translate("QTRehaZenter", "QT RehaZenter Exercise Assistant", None))
        self.btnBegin.setText(_translate("QTRehaZenter", "Begin!", None))
        self.btnStop.setText(_translate("QTRehaZenter", "Stop", None))
        self.lblROSConsole.setText(_translate("QTRehaZenter", "Robot log:", None))
        self.btnExternalRotationExercise.setText(_translate("QTRehaZenter", "External rotation exercise", None))
        self.btnAbductionMotionExercise.setText(_translate("QTRehaZenter", "Abduction motion exercise", None))
        self.btnInternalRotationExercise.setText(_translate("QTRehaZenter", "Internal rotation exercise", None))
        self.btnFlexionMotionExercise.setText(_translate("QTRehaZenter", "Flexion motion exercise", None))
        self.menuEdit.setTitle(_translate("QTRehaZenter", "File", None))
        self.menuHelp.setTitle(_translate("QTRehaZenter", "Help", None))
        self.toolBar.setWindowTitle(_translate("QTRehaZenter", "toolBar", None))
        self.toolBar_2.setWindowTitle(_translate("QTRehaZenter", "toolBar_2", None))
        self.actionPreferences.setText(_translate("QTRehaZenter", "Preferences", None))
        self.actionAbout.setText(_translate("QTRehaZenter", "About...", None))
        self.actionQuit.setText(_translate("QTRehaZenter", "Quit", None))


    # *******************************************************************************************
    # *************************  connector functions for the UI buttons  ************************
    # *******************************************************************************************
    def btnFlexionMotionExerciseClicked(self):
	# enable all other buttons except the one for exercise 1
	self.btnBegin.setEnabled(True)
	self.btnFlexionMotionExercise.setEnabled(False)
	self.btnAbductionMotionExercise.setEnabled(True)
	self.btnInternalRotationExercise.setEnabled(True)
	self.btnExternalRotationExercise.setEnabled(True)
	self.txtViewROSConsole.appendPlainText("Flexion motion exercise selected.")
	
    def btnAbductionMotionExerciseClicked(self):
	# enable all other buttons except the one for exercise 2
	self.btnBegin.setEnabled(True)
	self.btnFlexionMotionExercise.setEnabled(True)
	self.btnAbductionMotionExercise.setEnabled(False)
	self.btnInternalRotationExercise.setEnabled(True)
	self.btnExternalRotationExercise.setEnabled(True)
	self.txtViewROSConsole.appendPlainText("Abduction motion exercise selected.")

    def btnInternalRotationExerciseClicked(self):
	# enable all other buttons except the one for exercise 3
	self.btnBegin.setEnabled(True)
	self.btnFlexionMotionExercise.setEnabled(True)
	self.btnAbductionMotionExercise.setEnabled(True)
	self.btnInternalRotationExercise.setEnabled(False)
	self.btnExternalRotationExercise.setEnabled(True)
	self.txtViewROSConsole.appendPlainText("Internal rotation exercise selected.")

    def btnExternalRotationExerciseClicked(self):
	# enable all other buttons except the one for exercise 4
	self.btnBegin.setEnabled(True)
	self.btnFlexionMotionExercise.setEnabled(True)
	self.btnAbductionMotionExercise.setEnabled(True)
	self.btnInternalRotationExercise.setEnabled(True)
	self.btnExternalRotationExercise.setEnabled(False)
	self.txtViewROSConsole.appendPlainText("External rotation exercise selected.")

    def btnBeginClicked(self):
	# disable all other buttons while the chosen exercise is running
	self.btnFlexionMotionExercise.setEnabled(False)
	self.btnAbductionMotionExercise.setEnabled(False)
	self.btnInternalRotationExercise.setEnabled(False)
	self.btnExternalRotationExercise.setEnabled(False)
	self.btnBegin.setEnabled(False)
	self.btnStop.setEnabled(True)

	# spawn exercise subprocess
	self.txtViewROSConsole.appendPlainText("******************** BEGIN EXERCISE ********************")
        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
	launch_params = ['roslaunch', 'reha_game', 'Exercise_Launcher.launch']
	launch_params.extend(('width:='+str(self.exercise_width), 'height:='+str(self.exercise_height), 'color:='+self.exercise_color, 'number_of_repetitions:='+str(self.exercise_number_of_repetitions), "time_limit:="+str(self.exercise_time_limit)))
	#if self.btnFlexionMotionExercise.isChecked():
        #self.launcher = roslaunch.parent.ROSLaunchParent(uuid, ["./../../launch/Exercise_Launcher.launch"])
	self.exercise_process = Popen(launch_params, stdin=PIPE)
		
	# second approach to running exercise (not working atm, needs callback)
	#config = roslaunch.launch.ROSLaunchConfig()
	#config.add_roslaunch_file("./../../launch/Exercise_Launcher.launch")
	#self.launcher = roslaunch.launch.ROSLaunchRunner(uuid, config)

        #elif self.btnAbductionMotionExercise.isChecked():
        #        # TODO: implement start_game() of this exercise and create launch file
        #        self.launcher = roslaunch.parent.ROSLaunchParent(uuid, ["./../../launch/Exercise_Launcher.launch"])
        #elif self.btnInternalRotationExercise.isChecked():
        #        # TODO: implement start_game() of this exercise and create launch file
        #        self.launcher = roslaunch.parent.ROSLaunchParent(uuid, ["./../../launch/Exercise_Launcher.launch"])
        #else:
        #        # TODO: implement start_game() of this exercise and create launch file
        #        self.launcher = roslaunch.parent.ROSLaunchParent(uuid, ["./../../launch/Exercise_Launcher.launch"])
        #self.launcher.start()

    def btnStopClicked(self):
	# stop exercise subprocess and kill ROS console worker thread
	#self.exercise_process.communicate(input="q")
        self.exercise_process.terminate()
	self.exercise_process.wait()
	self.exercise_process = None

	# enable all other buttons again
	self.btnFlexionMotionExercise.setEnabled(True)
	self.btnAbductionMotionExercise.setEnabled(True)
	self.btnInternalRotationExercise.setEnabled(True)
	self.btnExternalRotationExercise.setEnabled(True)
	self.btnBegin.setEnabled(True)
	self.btnStop.setEnabled(False)
	self.txtViewROSConsole.appendPlainText("********************* END EXERCISE *********************")

    def appendToTextView(self, line):
	self.txtViewROSConsole.appendPlainText(line)

    def openPreferences(self):
	self.preferences.show()

    def updateExerciseParams(self, width, height, color, number_of_repetitions, time_limit):
	self.exercise_width = width
        self.exercise_height = height
        self.exercise_color = color
        self.exercise_number_of_repetitions = number_of_repetitions
        self.exercise_time_limit = time_limit

    def closeEvent(self, event):
        reply = QtGui.QMessageBox.question(self, 'Message',
            "Are you sure that you want to quit?", QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)

        if reply == QtGui.QMessageBox.Yes:
		if hasattr(self, "exercise_process") and self.exercise_process != None:
			#self.exercise_process.communicate(input="q")
			self.exercise_process.terminate() 
			self.exercise_process.wait()
		event.accept()
        else:
		event.ignore()

    # *******************************************************************************************

if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    myapp = QTRehaZenterGUI() 
    #app.processEvents()
    myapp.show()
    sys.exit(app.exec_())


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
from PyQt4 import uic
import Exercises
from Exercises import Color,Limb,RotationType,MotionType
import Preferences
import LoadCustomObject

class QTRehaZenterGUI(QtGui.QMainWindow):
    def __init__(self):
	super(QTRehaZenterGUI, self).__init__()
	uic.loadUi('ui_files/QTRehaZenterGUI.ui', self)

	# initialize exercise parameters
	self.exercise_width = 640
        self.exercise_height = 480
        self.exercise_color = "yellow"
	self.exercise_custom_color = None
        self.exercise_number_of_repetitions = 10
        self.exercise_time_limit = 0
	self.exercise_calibration_duration = 5

	# initialize other widgets
	self.preferences = Preferences.UIPreferencesWidget(self)
	self.loadCustomObject = LoadCustomObject.UILoadCustomObjectWidget(self)

	# connect functions to buttons
	self.btnInternalRotationExercise.clicked.connect(self.btnInternalRotationExerciseClicked)
	self.btnExternalRotationExercise.clicked.connect(self.btnExternalRotationExerciseClicked)
	self.btnAbductionMotionExercise.clicked.connect(self.btnAbductionMotionExerciseClicked)
	self.btnFlexionMotionExercise.clicked.connect(self.btnFlexionMotionExerciseClicked)
	self.btnBegin.clicked.connect(self.btnBeginClicked)
	self.btnStop.clicked.connect(self.btnStopClicked)
	self.actionQuit.triggered.connect(self.closeEvent)
	self.actionPreferences.triggered.connect(self.openPreferences)
	self.actionLoadCustomObject.triggered.connect(self.openLoadCustomObject)


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
	self.txtViewLogOutput.appendPlainText("Flexion motion exercise selected.")
	
    def btnAbductionMotionExerciseClicked(self):
	# enable all other buttons except the one for exercise 2
	self.btnBegin.setEnabled(True)
	self.btnFlexionMotionExercise.setEnabled(True)
	self.btnAbductionMotionExercise.setEnabled(False)
	self.btnInternalRotationExercise.setEnabled(True)
	self.btnExternalRotationExercise.setEnabled(True)
	self.txtViewLogOutput.appendPlainText("Abduction motion exercise selected.")

    def btnInternalRotationExerciseClicked(self):
	# enable all other buttons except the one for exercise 3
	self.btnBegin.setEnabled(True)
	self.btnFlexionMotionExercise.setEnabled(True)
	self.btnAbductionMotionExercise.setEnabled(True)
	self.btnInternalRotationExercise.setEnabled(False)
	self.btnExternalRotationExercise.setEnabled(True)
	self.txtViewLogOutput.appendPlainText("Internal rotation exercise selected.")

    def btnExternalRotationExerciseClicked(self):
	# enable all other buttons except the one for exercise 4
	self.btnBegin.setEnabled(True)
	self.btnFlexionMotionExercise.setEnabled(True)
	self.btnAbductionMotionExercise.setEnabled(True)
	self.btnInternalRotationExercise.setEnabled(True)
	self.btnExternalRotationExercise.setEnabled(False)
	self.txtViewLogOutput.appendPlainText("External rotation exercise selected.")

    def btnBeginClicked(self):
	# disable all other buttons while the chosen exercise is running
	self.btnFlexionMotionExercise.setEnabled(False)
	self.btnAbductionMotionExercise.setEnabled(False)
	self.btnInternalRotationExercise.setEnabled(False)
	self.btnExternalRotationExercise.setEnabled(False)
	self.btnBegin.setEnabled(False)
	self.btnStop.setEnabled(True)

	# spawn exercise subprocess
	self.txtViewLogOutput.appendPlainText("******************** BEGIN EXERCISE ********************")
        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
	launch_params = ['roslaunch', 'reha_game', 'Exercise_Launcher.launch']
	launch_params.extend(('width:='+str(self.exercise_width), 'height:='+str(self.exercise_height), 'color:='+self.exercise_color, 'number_of_repetitions:='+str(self.exercise_number_of_repetitions), "time_limit:="+str(self.exercise_time_limit)))
	#if self.btnFlexionMotionExercise.isChecked():
        #self.launcher = roslaunch.parent.ROSLaunchParent(uuid, ["./../../launch/Exercise_Launcher.launch"])
	self.exercise_process = Popen(launch_params)
		
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
	self.txtViewLogOutput.appendPlainText("********************* END EXERCISE *********************")

    def appendToTextView(self, line):
	self.txtViewLogOutput.appendPlainText(line)

    def openPreferences(self):
	self.preferences.show()

    def openLoadCustomObject(self):
	self.loadCustomObject.show()

    def updateExerciseParams(self, width, height, color, number_of_repetitions, time_limit, calibration_duration):
	self.exercise_width = width
        self.exercise_height = height
        self.exercise_color = color
	self.exercise_custom_color = None
        self.exercise_number_of_repetitions = number_of_repetitions
        self.exercise_time_limit = time_limit
	self.eexercise_calibration_duration = calibration_duration

    def updateCustomColor(self, custom_color):
	self.exercise_custom_color = custom_color

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


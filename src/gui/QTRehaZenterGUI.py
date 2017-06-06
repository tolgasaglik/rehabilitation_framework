# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'self.UI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import QThread, QRectF, Qt
from subprocess import Popen,PIPE
from PyQt4.QtGui import QWidget, QTabWidget, QLabel, QImage, QPixmap, QGraphicsScene, QGraphicsPixmapItem
import os,sys,inspect
#from threading import Thread
#from time import sleep
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
import DefineNewColor

class QTRehaZenterGUI(QtGui.QMainWindow):
    def __init__(self):
	super(QTRehaZenterGUI, self).__init__()
	uic.loadUi('ui_files/QTRehaZenterGUI.ui', self)

	# initialize custom object loader widget
	self.defineNewColorWidget = DefineNewColor.UIDefineNewColorWidget(self)

	# load logo images
	uniLuLogoScene = QGraphicsScene()
	imagePixmap_unilu = QGraphicsPixmapItem(QPixmap(QImage("imgs/university_of_luxembourg_logo.png")), None, uniLuLogoScene)
	self.grUniLuLogo.setScene(uniLuLogoScene)
	self.grUniLuLogo.fitInView(uniLuLogoScene.sceneRect(), Qt.KeepAspectRatio)
	rehaZenterLogoScene = QGraphicsScene()
	imagePixmap_reha = QGraphicsPixmapItem(QPixmap(QImage("imgs/rehazenter_logo.jpg")), None, rehaZenterLogoScene)
	self.grRehaZenterLogo.setScene(rehaZenterLogoScene)
	self.grRehaZenterLogo.fitInView(rehaZenterLogoScene.sceneRect(), Qt.KeepAspectRatio)

	# connect functions to buttons
	self.btnInternalRotationExercise.clicked.connect(self.btnInternalRotationExerciseClicked)
	self.btnExternalRotationExercise.clicked.connect(self.btnExternalRotationExerciseClicked)
	self.btnAbductionMotionExercise.clicked.connect(self.btnAbductionMotionExerciseClicked)
	self.btnFlexionMotionExercise.clicked.connect(self.btnFlexionMotionExerciseClicked)
	self.btnBegin.clicked.connect(self.btnBeginClicked)
	self.btnStop.clicked.connect(self.btnStopClicked)
	self.btnDefineNewColor.clicked.connect(self.openDefineNewColorWidget)
	self.actionQuit.triggered.connect(self.closeEvent)


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
	launch_params.extend(('width:='+str(self.spnWidth.value()), 'height:='+str(self.spnHeight.value()), 'color:=yellow', 'number_of_repetitions:='+str(self.spnNbrRepetitions.value()), "time_limit:="+str(self.spnTimeLimit.value()), 'calibration_duration:='+str(self.spnCalibDuration.value())))
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

    def openDefineNewColorWidget(self):
	self.defineNewColorWidget.show()

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


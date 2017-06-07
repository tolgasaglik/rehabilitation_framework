# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'self.UI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import QThread, QRectF, Qt
from subprocess import Popen,PIPE
from PyQt4.QtGui import QWidget, QTabWidget, QLabel, QImage, QPixmap, QGraphicsScene, QGraphicsPixmapItem, QHeaderView, QTableWidgetItem
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

	# initialize list of faces (in string form)
	self._faces_list = ["sad", "happy", "crying"]
	self.cmbFaces.addItems(self._faces_list)

	# disable various labels and widgets on startup
	self.lblPerRepetitions1.setEnabled(False)
	self.lblPerRepetitions2.setEnabled(False)
	self.spnQuantEncRep.setEnabled(False)
	self.btnDeleteLine.setEnabled(False)
	self.spnFixedReps.setEnabled(False)
	self.spnFrequencyReps.setEnabled(False)
	self.btnAddLine.setEnabled(False)

	# resize table columns to match their text size
	self.tblFacialFeedback.setColumnCount(3)
	header = self.tblFacialFeedback.horizontalHeader()
	header.setResizeMode(0, QHeaderView.Stretch)
	header.setResizeMode(1, QHeaderView.Stretch)
	header.setResizeMode(2, QHeaderView.Stretch)

	# connect functions to widgets
	self.btnInternalRotationExercise.clicked.connect(self.btnInternalRotationExerciseClicked)
	self.btnExternalRotationExercise.clicked.connect(self.btnExternalRotationExerciseClicked)
	self.btnAbductionMotionExercise.clicked.connect(self.btnAbductionMotionExerciseClicked)
	self.btnFlexionMotionExercise.clicked.connect(self.btnFlexionMotionExerciseClicked)
	self.btnBegin.clicked.connect(self.btnBeginClicked)
	self.btnStop.clicked.connect(self.btnStopClicked)
	self.btnDefineNewColor.clicked.connect(self.openDefineNewColorWidget)
	self.slNbrBlocks.valueChanged.connect(self.slNbrBlocksValueChanged)
	self.rdFixed.clicked.connect(self.rdFixedClicked)
	self.rdFrequency.clicked.connect(self.rdFrequencyClicked)
	self.chkQuantitative.clicked.connect(self.chkQuantitativeClicked)
	self.btnAddLine.clicked.connect(self.btnAddLineClicked)
	self.cmbFaces.currentIndexChanged.connect(self.cmbFacesCurrentIndexChanged)
	self.actionQuit.triggered.connect(self.closeEvent)
	self.btnDeleteLine.clicked.connect(self.btnDeleteLineClicked)
	self.tblFacialFeedback.itemClicked.connect(self.tblFacialFeedbackItemClicked)


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

    def chkQuantitativeClicked(self):
	self.chkQuantitative.setEnabled(not self.chkQuantitative.isChecked())

    def rdFixedClicked(self):
	self.spnFrequencyReps.setEnabled(False)
	self.spnFixedReps.setEnabled(True)
	if self.cmbFaces.currentIndex() > -1:
		self.btnAddLine.setEnabled(True)
	else:
		self.btnAddLine.setEnabled(False)	

    def rdFrequencyClicked(self):
	self.spnFrequencyReps.setEnabled(True)
	self.spnFixedReps.setEnabled(False)
	if self.cmbFaces.currentIndex() > -1:
		self.btnAddLine.setEnabled(True)
	else:
		self.btnAddLine.setEnabled(False)

    def btnDeleteLineClicked(self):
	self.tblFacialFeedback.removeRow(self.tblFacialFeedback.currentRow())
	self.btnDeleteLine.setEnabled(False)
	

    def tblFacialFeedbackItemClicked(self):
	self.btnDeleteLine.setEnabled(True)

    def btnAddLineClicked(self):
	self.tblFacialFeedback.insertRow(self.tblFacialFeedback.rowCount())
	if self.rdFixed.isChecked():
		typeText = "fixed"
		repetitionsText = str(self.spnFixedReps.value())
	elif self.rdFrequency.isChecked():
		typeText = "frequency"
		repetitionsText = str(self.spnFrequencyReps.value())
	else:
		raise Exception("error when selecting facial feedback, this is not supposed to happen...")
	faceText = self.cmbFaces.currentText()
	self.tblFacialFeedback.setItem(self.tblFacialFeedback.rowCount()-1, 0, QTableWidgetItem(typeText))
	self.tblFacialFeedback.setItem(self.tblFacialFeedback.rowCount()-1, 1, QTableWidgetItem(repetitionsText))
	self.tblFacialFeedback.setItem(self.tblFacialFeedback.rowCount()-1, 2, QTableWidgetItem(faceText))
	self.spnFixedReps.setEnabled(False)
	self.spnFrequencyReps.setEnabled(False)
	self.rdFixed.setChecked(False)
	self.rdFrequency.setChecked(False)
	self.cmbFaces.setCurrentIndex(-1)
	self.btnAddLine.setEnabled(False)

    def appendToTextView(self, line):
	self.txtViewLogOutput.appendPlainText(line)

    def cmbFacesCurrentIndexChanged(self):
	if self.rdFixed.isChecked() or self.rdFrequency.isChecked():
		self.btnAddLine.setEnabled(True)

    def openDefineNewColorWidget(self):
	self.defineNewColorWidget.show()

    def updateCustomColor(self, custom_color):
	self.exercise_custom_color = custom_color

    def slNbrBlocksValueChanged(self):
	self.lblNbrBlocksValue.setText(str(self.slNbrBlocks.value()))

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


# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'self.UI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import QThread, QRectF, Qt
from PyQt4.QtGui import QMessageBox, QFileDialog, QWidget, QTabWidget, QLabel, QImage, QPixmap, QGraphicsScene, QGraphicsPixmapItem, QHeaderView, QTableWidgetItem
import os,sys,inspect,ast
import rospy
# include parent "src" directory to sys.path, otherwise import won't work
# (source: http://stackoverflow.com/questions/714063/importing-modules-from-parent-folder)
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
from PyQt4 import uic
import Exercises
from Exercises import Limb,RotationType,MotionType,RobotPosition
from rehabilitation_framework.msg import *
from rehabilitation_framework.srv import *
import DefineNewColor

def load_color_file(filename):
	rgb_color_fileptr = open(filename, "r")
	colors = RGBColorList()
	for line in rgb_color_fileptr.readlines():
		to_array = ast.literal_eval(line)
		assert len(to_array) == 3
		color = RGBColor()
		color.red = to_array[0]
		color.green = to_array[1]
		color.blue = to_array[2]
		colors.rgb_color_list.append(color)
	rgb_color_fileptr.close()
	return colors


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

	# initialize calibration file selection dialog
	self.dlgLoadCalibFile = QFileDialog()
	self.dlgLoadCalibFile.setFileMode(QFileDialog.ExistingFile)
	self.dlgLoadCalibFile.setFilter("Calibration files (*.clb)")
	self.dlgLoadCalibFile.setAcceptMode(QFileDialog.AcceptOpen)

	# initialize color file selection dialog
	self.dlgLoadColorFile = QFileDialog()
	self.dlgLoadColorFile.setFileMode(QFileDialog.ExistingFile)
	self.dlgLoadColorFile.setFilter("Color files (*.clr)")
	self.dlgLoadColorFile.setAcceptMode(QFileDialog.AcceptOpen)

	# initialize calibration file save dialog
	self.dlgSaveCalibFile = QFileDialog()
	self.dlgSaveCalibFile.setFileMode(QFileDialog.AnyFile)
	self.dlgSaveCalibFile.setFilter("Calibration files (*.cal)")
	self.dlgSaveCalibFile.setAcceptMode(QFileDialog.AcceptSave)

	# initialize rotation exercises warning message box
	self.msgRotationExercises = QMessageBox()
	self.msgRotationExercises.setIcon(QMessageBox.Warning)
	self.msgRotationExercises.setText("Sorry, rotation exercises have not been implemented yet!")
	self.msgRotationExercises.setInformativeText("Please choose one of the motion exercises instead until rotation exercises become available.")
	self.msgRotationExercises.setWindowTitle("Rotation exercises warning")
	self.msgRotationExercises.setStandardButtons(QMessageBox.Ok)

	# initialize list of faces (in string form)
	self._faces_list = ["sad", "happy", "crying"]
	self.cmbFaces.addItems(self._faces_list)

	# disable various labels and widgets on startup
	self.lblPerRepetitions1.setEnabled(False)
	self.lblPerRepetitions2.setEnabled(False)
	self.spnQuantEncRep.setEnabled(False)
	self.cmbQualiEnc.setEnabled(False)
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
	self.btnAddLine.clicked.connect(self.btnAddLineClicked)
	self.cmbFaces.currentIndexChanged.connect(self.cmbFacesCurrentIndexChanged)
	self.actionQuit.triggered.connect(self.closeEvent)
	self.btnDeleteLine.clicked.connect(self.btnDeleteLineClicked)
	self.tblFacialFeedback.itemClicked.connect(self.tblFacialFeedbackItemClicked)
	self.chkQualitative.clicked.connect(self.chkQualitativeClicked)
	self.chkQuantitative.clicked.connect(self.chkQuantitativeClicked)
	self.btnLoadColorFile.clicked.connect(self.btnLoadColorFileClicked)
	self.btnLoadCalibFile.clicked.connect(self.btnLoadCalibFileClicked)
	self.btnCalibrateNow.clicked.connect(self.btnCalibrateNowClicked)


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
	# rotation exercises have not been implemented yet...
	if not self.btnInternalRotationExercise.isEnabled() or not self.btnExternalRotationExercise.isEnabled():
		self.msgRotationExercises.exec_()
		return

	# disable all other buttons while the chosen exercise is running
	self.btnFlexionMotionExercise.setEnabled(False)
	self.btnAbductionMotionExercise.setEnabled(False)
	self.btnInternalRotationExercise.setEnabled(False)
	self.btnExternalRotationExercise.setEnabled(False)
	self.btnBegin.setEnabled(False)
	self.btnStop.setEnabled(True)

	# spawn exercise subprocess
	self.txtViewLogOutput.appendPlainText("******************** BEGIN EXERCISE ********************")
	#launch_params = ['roslaunch', 'reha_game', 'Exercise_Launcher.launch']
	#launch_params.extend(('width:='+str(self.spnWidth.value()), 'height:='+str(self.spnHeight.value()), 'color:=yellow', 'number_of_repetitions:='+str(self.spnNbrRepetitions.value()), "time_limit:="+str(self.spnTimeLimit.value()), 'calibration_duration:='+str(self.spnCalibDuration.value())))
	pub = rospy.Publisher("Reha_ExerciseInit", ExerciseInit)
	rospy.init_node("Reha_ExerciseNode", anonymous=True)
	msg = ExerciseInit()

	# build exercise init message
	msg.camera_width = self.spnWidth.value()
	msg.camera_height = self.spnHeight.value()
	msg.blocks = self.slNbrBlocks.value()
	msg.repetitions = self.spnTimeLimit.value()
	msg.calibration_duration = self.spnCalibDuration.value()
	msg.quantitative_enabled = self.chkQuantitative.isChecked()
	msg.quantitative_frequency = self.spnQuantEncRep.value()
	msg.qualitative_enabled = self.chkQualitative.isChecked()
	msg.qualitative_frequency = self.spnQualiEnc.value()
	msg.robot_position = self.cmbRobotPosition.currentIndex()
	msg.rotation_type = 0
	

	#if self.btnFlexionMotionExercise.isChecked():
		#launch_params.extend(('motion_type:=0'))
	#elif self.btnAbductionMotionExercise.isChecked():
		#launch_params.extend(('motion_type:=1'))
	#self.exercise_process = Popen(launch_params)

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
	self.spnQuantEncRep.setEnabled(self.chkQuantitative.isChecked())

    def chkQualitativeClicked(self):
	self.cmbQualiEnc.setEnabled(self.chkQualitative.isChecked())

    def btnLoadColorFileClicked(self):
	if self.dlgLoadColorFile.exec_():
		filename = self.dlgLoadColorFile.selectedFiles()[0]
		self.lnColorFile.setText(filename)

    def btnLoadCalibFileClicked(self):
	if self.dlgLoadCalibFile.exec_():
		filename = self.dlgLoadCalibFile.selectedFiles()[0]
		self.lnCalibFile.setText(filename)

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

    def btnCalibrateNowClicked(self):
	if self.dlgSaveCalibFile.exec_():
		# create calibration service request message
		request = CalibrationRequest()
		if str(self.cmbCreateCalibFileFor.currentText()) == "flexion exercise":
			request.motion_type = 1
			request.rotation_type = 0
		elif str(self.cmbCreateCalibFileFor.currentText()) == "abduction exercise":
			request.motion_type = 1
			request.rotation_type = 0
		else:
			self.msgRotationExercises.exec_()
			return
		try:
			request.rgb_color_list = load_color_file(str(self.lnColorFile.text()))
		except IOError:
			# TODO: maybe show error in dialog box instead?
			print "Color file could not be read!"
			return
		except ValueError:
			# TODO: maybe show error in dialog box instead?
			print "Color file has invalid contents!"
			return
		request.robot_position = self.cmbRobotPosition.currentIndex()
		request.calibration_duration = self.spnCalibDuration.value()
		request.camera_width = self.spnWidth.value()
		request.camera_height = self.spnHeight.value()

		# publish request to service
		rospy.wait_for_service('calibrate')
		try:
			calibrate = rospy.ServiceProxy('calibrate', Calibration)
			response = calibrate(request)
		except rospy.ServiceException, e:
			# TODO: maybe show error in dialog box instead?
			print "Service call failed: %s"%e
			return

		# write calibration points to file
		filename = self.dlgSaveCalibFile.selectedFiles()[0]
		if not filename.endsWith(".clb"):
			filename += ".clb"
		calib_fileptr = open(filename, "w")
		calib_fileptr.write("motion_type=" + str(request.motion_type) + "\n")
		calib_fileptr.write("rotation_type=" + str(request.rotation_type) + "\n")
		calib_fileptr.write("robot_position=" + str(request.robot_position) + "\n")
		calib_fileptr.write("calibration_points_left_arm=" + str(response.calibration_points_left_arm)+ "\n")
		calib_fileptr.write("calibration_points_right_arm=" + str(response.calibration_points_right_arm)+ "\n")
		calib_fileptr.close()
	

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

    def updateColorFileName(self, color_filename):
	self.lnColorFile.setText(color_filename)

    def slNbrBlocksValueChanged(self):
	self.lblNbrBlocksValue.setText(str(self.slNbrBlocks.value()))

    def closeEvent(self, event):
        reply = QtGui.QMessageBox.question(self, 'Message', "Are you sure that you want to quit?", QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
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


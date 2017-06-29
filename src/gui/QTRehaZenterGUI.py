# -*- coding: utf-8 -*)-

# Form implementation generated from reading ui file 'self.UI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import QThread, QRectF, Qt, pyqtSignal, pyqtSlot
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
from std_msgs.msg import Bool
import DefineNewColor

# some helper functions
def load_color_file(filename):
    rgb_color_fileptr = open(filename, "r")
    colors = []
    for line in rgb_color_fileptr.readlines():
        to_array = ast.literal_eval(line)
        assert len(to_array) == 3
        color = RGBColor()
        color.red = to_array[0]
        color.green = to_array[1]
        color.blue = to_array[2]
        colors.append(color)
    rgb_color_fileptr.close()
    return colors

def load_calib_file(filename):
    calib_fileptr = open(filename, "r")
    calibration_points_left_arm = []
    calibration_points_right_arm = []
    for line in calib_fileptr,readline():
        if line.startswith("left_arm="):
            cb_points_from_file = ast.literal_eval(line[9:])
        elif line.startswith("right_arm="):
                cb_points_from_file = ast.literal_eval(line[10:])
        else:
            raise ValueError("Invalid file contents!")
    for cb_point in cb_points_from_file:
        point_to_add = CalibrationPoint()
        point_to_add.x = cb_point[0]
        point_to_add.y = cb_point[1]
        if line.startswith("left_arm="):
            calibration_points_left_arm.append(point_to_add)
        else:
            calibration_points_right_arm.append(point_to_add)
    calib_fileptr.close()
    return (calibration_points_left_arm, calibration_points_right_arm)

@pyqtSlot(object, int)
def robot_finished_triggered(gui, status):
    if status != 0:
        gui.msgErrorWarning.setText("Calibration was interrupted! Please try to calibrate again.")
        gui.msgErrorWarning.setWindowTitle("Calibration interrupted")
        gui.msgErrorWarning.exec_()
    gui.enableAllWidgets()
#------------------------

class QTRehaZenterGUI(QtGui.QMainWindow):
    robot_finished = pyqtSignal(object, int, name="robot_finished")
    _save_calib_filename = ""
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
        self.dlgSaveCalibFile.setFilter("Calibration files (*.clb)")
        self.dlgSaveCalibFile.setAcceptMode(QFileDialog.AcceptSave)
        
        # initialize rotation exercises warning message box
        self.msgRotationExercises = QMessageBox()
        self.msgRotationExercises.setIcon(QMessageBox.Warning)
        self.msgRotationExercises.setText("Sorry, rotation exercises have not been implemented yet!")
        self.msgRotationExercises.setInformativeText("Please choose one of the motion exercises instead until rotation exercises become available.")
        self.msgRotationExercises.setWindowTitle("Rotation exercises warning")
        self.msgRotationExercises.setStandardButtons(QMessageBox.Ok)
        
        # initialize calibration fail message box
        self.msgErrorWarning = QMessageBox()
        self.msgErrorWarning.setIcon(QMessageBox.Warning)
        self.msgErrorWarning.setStandardButtons(QMessageBox.Ok)
        
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
        header = self.tblEmotionalFeedback.horizontalHeader()
        header.setResizeMode(0, QHeaderView.Stretch)
        header.setResizeMode(1, QHeaderView.Stretch)
        header.setResizeMode(2, QHeaderView.Stretch)
        
        # initialize ROS publisher topics
        rospy.init_node("reha_interface", anonymous=True)
        self._exercise_init_pub = rospy.Publisher("exercise_init", ExerciseInit, queue_size=1)
        self._exercise_stop_pub = rospy.Publisher("exercise_stop", Bool, queue_size=1)
        self._calibration_request_pub = rospy.Publisher("calibration_request", CalibrationRequest, queue_size=1)
        rospy.Subscriber("calibration_reply", CalibrationReply, self._calibration_reply_callback)

    # initialize some other necessary variables
        self._calibrate_only = False
        
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
        self.tblEmotionalFeedback.itemClicked.connect(self.tblEmotionalFeedbackItemClicked)
        self.chkQualitative.clicked.connect(self.chkQualitativeClicked)
        self.chkQuantitative.clicked.connect(self.chkQuantitativeClicked)
        self.btnLoadColorFile.clicked.connect(self.btnLoadColorFileClicked)
        self.btnLoadCalibFile.clicked.connect(self.btnLoadCalibFileClicked)
        self.btnCalibrateNow.clicked.connect(self.btnCalibrateNowClicked)
        self.robot_finished.connect(robot_finished_triggered)
    
    # **** some helper functions specific to the class ****
    def disableAllWidgets(self):
        # disable all other buttons while the chosen exercise is running
        self.btnFlexionMotionExercise.setEnabled(False)
        self.btnAbductionMotionExercise.setEnabled(False)
        self.btnInternalRotationExercise.setEnabled(False)
        self.btnExternalRotationExercise.setEnabled(False)
        self.btnBegin.setEnabled(False)
        self.btnStop.setEnabled(True)
        self.slNbrBlocks.setEnabled(False)
        self.spnNbrRepetitions.setEnabled(False)
        self.spnTimeLimit.setEnabled(False)
        self.chkQualitative.setEnabled(False)
        self.chkQuantitative.setEnabled(False)
        self.spnQuantEncRep.setEnabled(False)
        self.cmbQualiEnc.setEnabled(False)
        self.btnDeleteLine.setEnabled(False)
        self.rdFixed.setEnabled(False)
        self.rdFrequency.setEnabled(False)
        self.spnFixedReps.setEnabled(False)
        self.spnFrequencyReps.setEnabled(False)
        self.cmbFaces.setEnabled(False)
        self.btnAddLine.setEnabled(False)
        self.spnWidth.setEnabled(False)
        self.spnHeight.setEnabled(False)
        self.cmbRobotPosition.setEnabled(False)
        self.spnCalibDuration.setEnabled(False)
        self.btnLoadColorFile.setEnabled(False)
        self.btnDefineNewColor.setEnabled(False)
        self.btnLoadCalibFile.setEnabled(False)
        self.cmbCreateCalibFileFor.setEnabled(False)
        self.btnCalibrateNow.setEnabled(False)

    def enableAllWidgets(self):
        # disable all other buttons while the chosen exercise is running
        self.btnFlexionMotionExercise.setEnabled(True)
        self.btnAbductionMotionExercise.setEnabled(True)
        self.btnInternalRotationExercise.setEnabled(True)
        self.btnExternalRotationExercise.setEnabled(True)
        self.btnBegin.setEnabled(False)
        self.btnStop.setEnabled(False)
        self.slNbrBlocks.setEnabled(True)
        self.spnNbrRepetitions.setEnabled(True)
        self.spnTimeLimit.setEnabled(True)
        self.chkQualitative.setEnabled(True)
        self.chkQuantitative.setEnabled(True)
        self.spnQuantEncRep.setEnabled(False)
        self.cmbQualiEnc.setEnabled(False)
        if self.tblEmotionalFeedback.rowCount() > 0:
            self.btnDeleteLine.setEnabled(True)
        else:
            self.btnDeleteLine.setEnabled(False)
        self.rdFixed.setEnabled(True)
        self.rdFrequency.setEnabled(True)
        self.spnFixedReps.setEnabled(False)
        self.spnFrequencyReps.setEnabled(False)
        self.btnAddLine.setEnabled(False)
        self.cmbFaces.setEnabled(True)
        self.btnAddLine.setEnabled(True)
        self.spnWidth.setEnabled(True)
        self.spnHeight.setEnabled(True)
        self.cmbRobotPosition.setEnabled(True)
        self.spnCalibDuration.setEnabled(True)
        self.btnLoadColorFile.setEnabled(True)
        self.btnDefineNewColor.setEnabled(True)
        self.btnLoadCalibFile.setEnabled(True)
        self.cmbCreateCalibFileFor.setEnabled(True)
        if self.lnColorFile.text() != "":
            self.btnCalibrateNow.setEnabled(True)
        else:
            self.btnCalibrateNow.setEnabled(False)
        self.lblPerRepetitions1.setEnabled(False)
        self.lblPerRepetitions2.setEnabled(False)
    
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
        self._calibrate_only = False
        
        # spawn exercise subprocess
        self.txtViewLogOutput.appendPlainText("******************** BEGIN EXERCISE ********************")
        msg = ExerciseInit()
        
        # build exercise init message
        msg.camera_width = self.spnWidth.value()
        msg.camera_height = self.spnHeight.value()
        msg.blocks = self.slNbrBlocks.value()
        msg.repetitions = self.spnTimeLimit.value()
        msg.calibration_duration = self.spnCalibDuration.value()
        if self.chkQuantitative.isChecked():
            msg.quantitative_frequency = self.spnQuantEncRep.value()
        else:
            msg.quantitative_frequency = 0
        if self.chkQualitative.isChecked():
            if self.cmbQualiEnc.text() == "high":
                msg.qualitative_frequency = 3
            elif self.cmbQualiEnc.text() == "medium":
                msg.qualitative_frequency = 2
            else:
                msg.qualitative_frequency = 1
        else:
            msg.qualitative_frequency = 0
            msg.qualitative_frequency = self.spnQualiEnc.value()
        msg.robot_position = self.cmbRobotPosition.currentIndex()
        msg.rotation_type = 0
        msg.emotional_feedback_list = []
        for i in range(0,self.tblEmotionalFeedback.columnCount()):
            emotional_feedback = EmotionalFeedback()
            emotional_feedback.is_fixed_feedback = (str(self.tblEmotionalFeedback.item(i, 0).text()) == "fixed")
            emotional_feedback.repetitions = int(self.tblEmotionalFeedback.item(i, 1).text())
            emotional_feedback.face_to_show = str(self.tblEmotionalFeedback.item(i, 0).text())
            msg.emotional_feedback_list.append(emotional_feedback)
        # TODO: show error dialog if files fail to load
        try:
            msg.rgb_colors = load_color_file(str(self.lnColorFile.text()))
        except ValueError:
            self.msgErrorWarning.setText("The specified color file has invalid contents!")
            self.msgErrorWarning.setWindowTitle("Invalid color file")
            self.msgErrorWarning.exec_()
            return
        except IOError:
            self.msgErrorWarning.setText("The specified color file could not be read! (does it exist?)")
            self.msgErrorWarning.setWindowTitle("Could not read color file")
            self.msgErrorWarning.exec_()
            return
        try:
            calib_data = load_calib_file(self.lnCalibFile.text())
        except ValueError:
            self.msgErrorWarning.setText("The specified calibration file has invalid contents!")
            self.msgErrorWarning.setWindowTitle("Invalid calibration file")
            self.msgErrorWarning.exec_()
            return
        except IOError:
            self.msgErrorWarning.setText("The specified calibration file could not be read! (does it exist?)")
            self.msgErrorWarning.setWindowTitle("Could not read calibration file")
            self.msgErrorWarning.exec_()
            return
        msg.calibration_points_left_arm = calib_data[0]
        msg.calibration_points_right_arm = calib_data[1]
        self._exercise_init_pub.publish(msg)
        self.txtViewLogOutput.appendPlainText("Current exercise configuration:\n" + str(msg))
        self.disableAllWidgets()

    def btnStopClicked(self):
        # stop exercise on robot
        self._exercise_stop_pub.publish(self._calibrate_only)

        # enable all other buttons again
        self.enableAllWidgets()
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
        self.tblEmotionalFeedback.removeRow(self.tblEmotionalFeedback.currentRow())
        self.btnDeleteLine.setEnabled(False)
     
    def btnCalibrateNowClicked(self):
        if self.dlgSaveCalibFile.exec_():
        # create calibration service request message
            self._calibration_only = True
            request = CalibrationRequest()
            if str(self.cmbCreateCalibFileFor.currentText()) == "flexion exercise":
                request.motion_type = 1
                request.rotation_type = 0
            elif str(self.cmbCreateCalibFileFor.currentText()) == "abduction exercise":
                request.motion_type = 2
                request.rotation_type = 0
            else:
                self.msgRotationExercises.exec_()
                return
            try:
                request.rgb_color_list = load_color_file(str(self.lnColorFile.text()))
            except ValueError:
                self.msgErrorWarning.setText("The specified color file has invalid contents!")
                self.msgErrorWarning.setWindowTitle("Invalid color file")
                self.msgErrorWarning.exec_()
                return
            except IOError:
                self.msgErrorWarning.setText("The specified color file could not be read! (does it exist?)")
                self.msgErrorWarning.setWindowTitle("Could not read color file")
                self.msgErrorWarning.exec_()
                return
            try:
                calib_data = load_calib_file(self.lnCalibFile.text())
            except ValueError:
                self.msgErrorWarning.setText("The specified calibration file has invalid contents!")
                self.msgErrorWarning.setWindowTitle("Invalid calibration file")
                self.msgErrorWarning.exec_()
                return
            except IOError:
                self.msgErrorWarning.setText("The specified calibration file could not be read! (does it exist?)")
                self.msgErrorWarning.setWindowTitle("Could not read calibration file")
                self.msgErrorWarning.exec_()
                return
            request.robot_position = self.cmbRobotPosition.currentIndex()
            request.calibration_duration = self.spnCalibDuration.value()
            request.camera_width = self.spnWidth.value()
            request.camera_height = self.spnHeight.value()
            self._save_calib_filename = self.dlgSaveCalibFile.selectedFiles()[0]

            # publish request to topic
            self._last_calib_request_msg = request
            self.disableAllWidgets()
            self.tabWidget.setCurrentIndex(1)
            self._calibration_request_pub.publish(request)
            self.txtViewLogOutput.appendPlainText("Current calibration configuration:\n" + str(request))

    def tblEmotionalFeedbackItemClicked(self):
        self.btnDeleteLine.setEnabled(True)

    def btnAddLineClicked(self):
        self.tblEmotionalFeedback.insertRow(self.tblEmotionalFeedback.rowCount())
        if self.rdFixed.isChecked():
            typeText = "fixed"
            repetitionsText = str(self.spnFixedReps.value())
        elif self.rdFrequency.isChecked():
            typeText = "frequency"
            repetitionsText = str(self.spnFrequencyReps.value())
        else:
            raise Exception("error when selecting facial feedback, this is not supposed to happen...")
        faceText = self.cmbFaces.currentText()
        self.tblEmotionalFeedback.setItem(self.tblEmotionalFeedback.rowCount()-1, 0, QTableWidgetItem(typeText))
        self.tblEmotionalFeedback.setItem(self.tblEmotionalFeedback.rowCount()-1, 1, QTableWidgetItem(repetitionsText))
        self.tblEmotionalFeedback.setItem(self.tblEmotionalFeedback.rowCount()-1, 2, QTableWidgetItem(faceText))
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

    def _calibration_reply_callback(self, data):
        # write calibration points to file
        if data.status == 0:
            if not self._save_calib_filename.endsWith(".clb"):
                self._save_calib_filename += ".clb"
            calib_fileptr = open(self._save_calib_filename, "w")
            if str(self.cmbCreateCalibFileFor.currentText()) == "flexion exercise":
                calib_fileptr.write("motion_type=1\n")
                calib_fileptr.write("rotation_type=0\n")
            elif str(self.cmbCreateCalibFileFor.currentText()) == "abduction exercise":
                calib_fileptr.write("motion_type=2\n")
                calib_fileptr.write("rotation_type=0\n")
            calib_fileptr.write("robot_position=" + str(self.cmbRobotPosition.currentIndex()) + "\n")
            calib_fileptr.write("calibration_points_left_arm=[")
            for point in data.calibration_points_left_arm:
                calib_fileptr.write("(" + str(point.x) + "," + str(point.y) + ")")
            calib_fileptr.write("]\n")
            calib_fileptr.write("calibration_points_right_arm=[")
            for point in data.calibration_points_right_arm:
                calib_fileptr.write("(" + str(point.x) + "," + str(point.y) + ")")
            calib_fileptr.write("]\n")
            calib_fileptr.close()
        self._is_calibrating = False
        self.robot_finished.emit(self, data.status)

        
# *******************************************************************************************
         
if __name__ == "__main__":
    from xmlrpclib import ServerProxy
    try:
        m = ServerProxy(os.environ['ROS_MASTER_URI'])
        code, msg, val = m.getSystemState("reha_interface")
    except:
        print("Unable to communicate with ROS master! Aborting.")
        sys.exit()
    app = QtGui.QApplication(sys.argv)
    myapp = QTRehaZenterGUI()
    #app.processEvents()
    myapp.show()
    sys.exit(app.exec_())

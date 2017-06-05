# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'QTRehaZenterGUI_Preferences.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui, uic


class UIPreferencesWidget(QtGui.QWidget):
    def __init__(self, main_window_ref):
	super(UIPreferencesWidget, self).__init__()
        uic.loadUi('ui_files/Preferences.ui', self)

        # store original data from main window as a backup
	self._main_window_ref = main_window_ref
        self._original_width = main_window_ref.exercise_width
        self._original_height = main_window_ref.exercise_height
        self._original_nbr_repetitions= main_window_ref.exercise_number_of_repetitions
        self._original_time_limit = main_window_ref.exercise_time_limit
        self._original_color = main_window_ref.exercise_color
	self._original_calib_duration = main_window_ref.exercise_calibration_duration

	# initialize value of widgets
        self.spnWidth.setValue(self._original_width)
        self.spnHeight.setValue(self._original_height)
        self.spnNbrRepetitions.setValue(self._original_nbr_repetitions)
        self.spnTimeLimit.setValue(self._original_time_limit)
	self.cmbColor.setCurrentIndex(self.cmbColor.findText(self._original_color))
	self.spnCalibDuration.setValue(self._original_calib_duration)

	# connect functions to buttons
        self.btnSaveChanges.clicked.connect(self.btnSaveChangesClicked)
        self.btnCancel.clicked.connect(self.btnCancelClicked)


    def btnSaveChangesClicked(self):
        self._main_window_ref.updateExerciseParams(self.spnWidth.value(), self.spnHeight.value(), str(self.cmbColor.currentText()), self.spnNbrRepetitions.value(), self.spnTimeLimit.value(), self.spnCalibDuration.value())
        
        self._original_width = self.spnWidth.value()
        self._original_height = self.spnHeight.value()
        self._original_nbr_repetitions = self.spnNbrRepetitions.value()
        self._original_time_limit = self.spnTimeLimit.value()
        self._original_color = str(self.cmbColor.currentText())
	self._original_calib_duration = self.spnCalibDuration.value()
	self.close()

    def btnCancelClicked(self):
	# replace widget values with the original data
        self.spnWidth.setValue(self._original_width)
        self.spnHeight.setValue(self._original_height)
        self.spnNbrRepetitions.setValue(self._original_nbr_repetitions)
        self.spnTimeLimit.setValue(self._original_time_limit)
        self.cmbColor.setCurrentIndex(self.cmbColor.findText(self._original_color))
        self.close()

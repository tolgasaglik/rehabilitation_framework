# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'QTRehaZenterGUI_Preferences.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
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

class UIPreferencesWidget(QtGui.QWidget):
    def __init__(self, main_window_ref):
	QtGui.QWidget.__init__(self)
        self.setupUi()
	self._main_window_ref = main_window_ref

        # store original data from main window as a backup
        self._original_width = main_window_ref.exercise_width
        self._original_height = main_window_ref.exercise_height
        self._original_nbr_repetitions= main_window_ref.exercise_number_of_repetitions
        self._original_time_limit = main_window_ref.exercise_time_limit
        self._original_color = main_window_ref.exercise_color

	# initialize value of widgets
        self.spnWidth.setValue(self._original_width)
        self.spnHeight.setValue(self._original_height)
        self.spnNbrRepetitions.setValue(self._original_nbr_repetitions)
        self.spnTimeLimit.setValue(self._original_time_limit)
	self.cmbColor.setCurrentIndex(self.cmbColor.findText(self._original_color))

    def setupUi(self):
        self.setObjectName(_fromUtf8("preferencesWidget"))
        self.resize(668, 499)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(sizePolicy)
        self.setMinimumSize(QtCore.QSize(668, 499))
        self.setMaximumSize(QtCore.QSize(668, 499))
        self.gridLayout = QtGui.QGridLayout(self)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.grpExerciseSettings = QtGui.QGroupBox(self)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.grpExerciseSettings.sizePolicy().hasHeightForWidth())
        self.grpExerciseSettings.setSizePolicy(sizePolicy)
        self.grpExerciseSettings.setObjectName(_fromUtf8("grpExerciseSettings"))
        self.gridLayout_3 = QtGui.QGridLayout(self.grpExerciseSettings)
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.cmbColor = QtGui.QComboBox(self.grpExerciseSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cmbColor.sizePolicy().hasHeightForWidth())
        self.cmbColor.setSizePolicy(sizePolicy)
        self.cmbColor.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.cmbColor.setObjectName(_fromUtf8("cmbColor"))
        self.cmbColor.addItem(_fromUtf8(""))
        self.cmbColor.addItem(_fromUtf8(""))
        self.cmbColor.addItem(_fromUtf8(""))
        self.gridLayout_3.addWidget(self.cmbColor, 6, 1, 1, 1)
        self.lblTimeLimit = QtGui.QLabel(self.grpExerciseSettings)
        self.lblTimeLimit.setObjectName(_fromUtf8("lblTimeLimit"))
        self.gridLayout_3.addWidget(self.lblTimeLimit, 4, 0, 1, 1)
        self.lblColor = QtGui.QLabel(self.grpExerciseSettings)
        self.lblColor.setObjectName(_fromUtf8("lblColor"))
        self.gridLayout_3.addWidget(self.lblColor, 6, 0, 1, 1)
        self.spnTimeLimit = QtGui.QSpinBox(self.grpExerciseSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.spnTimeLimit.sizePolicy().hasHeightForWidth())
        self.spnTimeLimit.setSizePolicy(sizePolicy)
        self.spnTimeLimit.setMinimum(0)
        self.spnTimeLimit.setMaximum(7200)
        self.spnTimeLimit.setObjectName(_fromUtf8("spnTimeLimit"))
        self.gridLayout_3.addWidget(self.spnTimeLimit, 4, 1, 1, 1)
        self.spnNbrRepetitions = QtGui.QSpinBox(self.grpExerciseSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.spnNbrRepetitions.sizePolicy().hasHeightForWidth())
        self.spnNbrRepetitions.setSizePolicy(sizePolicy)
        self.spnNbrRepetitions.setMinimum(1)
        self.spnNbrRepetitions.setMaximum(100)
        self.spnNbrRepetitions.setObjectName(_fromUtf8("spnNbrRepetitions"))
        self.gridLayout_3.addWidget(self.spnNbrRepetitions, 0, 1, 1, 1)
        self.lblNbrRepetitions = QtGui.QLabel(self.grpExerciseSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lblNbrRepetitions.sizePolicy().hasHeightForWidth())
        self.lblNbrRepetitions.setSizePolicy(sizePolicy)
        self.lblNbrRepetitions.setObjectName(_fromUtf8("lblNbrRepetitions"))
        self.gridLayout_3.addWidget(self.lblNbrRepetitions, 0, 0, 1, 1)
        self.lblCalibDuration = QtGui.QLabel(self.grpExerciseSettings)
        self.lblCalibDuration.setObjectName(_fromUtf8("lblCalibDuration"))
        self.gridLayout_3.addWidget(self.lblCalibDuration, 5, 0, 1, 1)
        self.spnCalibDuration = QtGui.QSpinBox(self.grpExerciseSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.spnCalibDuration.sizePolicy().hasHeightForWidth())
        self.spnCalibDuration.setSizePolicy(sizePolicy)
        self.spnCalibDuration.setMinimum(5)
        self.spnCalibDuration.setMaximum(30)
        self.spnCalibDuration.setObjectName(_fromUtf8("spnCalibDuration"))
        self.gridLayout_3.addWidget(self.spnCalibDuration, 5, 1, 1, 1)
        self.gridLayout.addWidget(self.grpExerciseSettings, 2, 0, 1, 3)
        self.grpCameraSettings = QtGui.QGroupBox(self)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.grpCameraSettings.sizePolicy().hasHeightForWidth())
        self.grpCameraSettings.setSizePolicy(sizePolicy)
        self.grpCameraSettings.setFlat(False)
        self.grpCameraSettings.setObjectName(_fromUtf8("grpCameraSettings"))
        self.gridLayout_2 = QtGui.QGridLayout(self.grpCameraSettings)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.spnHeight = QtGui.QSpinBox(self.grpCameraSettings)
        self.spnHeight.setMaximumSize(QtCore.QSize(120, 40))
        self.spnHeight.setMinimum(1)
        self.spnHeight.setMaximum(1080)
        self.spnHeight.setObjectName(_fromUtf8("spnHeight"))
        self.gridLayout_2.addWidget(self.spnHeight, 2, 1, 1, 1)
        self.spnWidth = QtGui.QSpinBox(self.grpCameraSettings)
        self.spnWidth.setMaximumSize(QtCore.QSize(120, 40))
        self.spnWidth.setMinimum(1)
        self.spnWidth.setMaximum(1920)
        self.spnWidth.setObjectName(_fromUtf8("spnWidth"))
        self.gridLayout_2.addWidget(self.spnWidth, 0, 1, 1, 1)
        self.lblWidth = QtGui.QLabel(self.grpCameraSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lblWidth.sizePolicy().hasHeightForWidth())
        self.lblWidth.setSizePolicy(sizePolicy)
        self.lblWidth.setObjectName(_fromUtf8("lblWidth"))
        self.gridLayout_2.addWidget(self.lblWidth, 0, 0, 1, 1)
        self.lblHeight = QtGui.QLabel(self.grpCameraSettings)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lblHeight.sizePolicy().hasHeightForWidth())
        self.lblHeight.setSizePolicy(sizePolicy)
        self.lblHeight.setObjectName(_fromUtf8("lblHeight"))
        self.gridLayout_2.addWidget(self.lblHeight, 2, 0, 1, 1)
        self.gridLayout.addWidget(self.grpCameraSettings, 0, 0, 1, 3)
        self.btnCancel = QtGui.QPushButton(self)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnCancel.sizePolicy().hasHeightForWidth())
        self.btnCancel.setSizePolicy(sizePolicy)
        self.btnCancel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.btnCancel.setObjectName(_fromUtf8("btnCancel"))
        self.gridLayout.addWidget(self.btnCancel, 3, 0, 1, 1)
        self.btnSaveChanges = QtGui.QPushButton(self)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnSaveChanges.sizePolicy().hasHeightForWidth())
        self.btnSaveChanges.setSizePolicy(sizePolicy)
        self.btnSaveChanges.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.btnSaveChanges.setObjectName(_fromUtf8("btnSaveChanges"))
        self.gridLayout.addWidget(self.btnSaveChanges, 3, 2, 1, 1)

        self.retranslateUi(self)
        QtCore.QMetaObject.connectSlotsByName(self)

	# connect functions to buttons
        self.btnSaveChanges.clicked.connect(self.btnSaveChangesClicked)
        self.btnCancel.clicked.connect(self.btnCancelClicked)


    def retranslateUi(self, preferencesWidget):
        preferencesWidget.setWindowTitle(_translate("preferencesWidget", "Preferences", None))
        self.grpExerciseSettings.setTitle(_translate("preferencesWidget", "Exercise settings", None))
        self.cmbColor.setItemText(0, _translate("preferencesWidget", "yellow", None))
        self.cmbColor.setItemText(1, _translate("preferencesWidget", "blue", None))
        self.cmbColor.setItemText(2, _translate("preferencesWidget", "black", None))
        self.lblTimeLimit.setText(_translate("preferencesWidget", "Time limit:", None))
        self.lblColor.setText(_translate("preferencesWidget", "Color:", None))
        self.lblNbrRepetitions.setText(_translate("preferencesWidget", "Number of repetitions:", None))
        self.lblCalibDuration.setText(_translate("preferencesWidget", "Calibration duration:", None))
        self.grpCameraSettings.setTitle(_translate("preferencesWidget", "Camera settings", None))
        self.lblWidth.setText(_translate("preferencesWidget", "Width:", None))
        self.lblHeight.setText(_translate("preferencesWidget", "Height:", None))
        self.btnCancel.setText(_translate("preferencesWidget", "Cancel", None))
        self.btnSaveChanges.setText(_translate("preferencesWidget", "Save changes", None))

    def btnSaveChangesClicked(self):
        self._main_window_ref.updateExerciseParams(self.spnWidth.value(), self.spnHeight.value(), str(self.cmbColor.currentText()), self.spnNbrRepetitions.value(), self.spnTimeLimit.value())
        
        self._original_width = self.spnWidth.value()
        self._original_height = self.spnHeight.value()
        self._original_nbr_repetitions = self.spnNbrRepetitions.value()
        self._original_time_limit = self.spnTimeLimit.value()
        self._original_color = str(self.cmbColor.currentText())
	self.close()

    def btnCancelClicked(self):
	# replace widget values with the original data
        self.spnWidth.setValue(self._original_width)
        self.spnHeight.setValue(self._original_height)
        self.spnNbrRepetitions.setValue(self._original_nbr_repetitions)
        self.spnTimeLimit.setValue(self._original_time_limit)
        self.cmbColor.setCurrentIndex(self.cmbColor.findText(self._original_color))
        self.close()

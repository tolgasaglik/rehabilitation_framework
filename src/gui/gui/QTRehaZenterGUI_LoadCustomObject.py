from PyQt4 import uic
from PyQt4.QtCore import QPoint
from PyQt4.QtGui import QColor, QSizePolicy, QPainter, QWidget, QFileDialog, QGraphicsScene, QGraphicsView, QImage, QBrush, QPixmap, QGraphicsPixmapItem

# taken from: https://stackoverflow.com/questions/4624985/how-simply-display-a-qcolor-using-pyqt
# ()
class ColorDisplayWidget(QWidget):
	def __init__(self, parent):
		super(ColorDisplay)

class UILoadCustomObjectWidget(QWidget):
	def __init__(self, main_window_ref):
		super(UILoadCustomObjectWidget, self).__init__()
		uic.loadUi('QTRehaZenterGUI_LoadCustomObject.ui', self)
		self._main_window_ref = main_window_ref
		self._MAX_COLORS = 10

		# initialize image selection dialog
		self.dlgLoadImage = QFileDialog()
		self.dlgLoadImage.setFileMode(QFileDialog.ExistingFile)
		self.dlgLoadImage.setFilter("Image files (*.png *.jpeg *.jpg *.bmp)")
		self.dlgLoadImage.setAcceptMode(QFileDialog.AcceptOpen)

		# initialize color saving dialog
                self.dlgSaveColors = QFileDialog()
                self.dlgSaveColors.setFileMode(QFileDialog.AnyFile)
                self.dlgSaveColors.setFilter("Image files (*.png *.jpeg *.jpg *.bmp)")
                self.dlgSaveColors.setAcceptMode(QFileDialog.AcceptSave)

		# initialize image graphics view
		self._imageScene = QGraphicsScene()
		self._detected_colors = []
		self._detected_colors_index = -1

		# make QGraphicsView objects invisible on startup

		widgets = (self.gridLayoutColors.itemAt(i).widget() for i in range(self.gridLayoutColors.count()))
                for w in widgets:
			if isinstance(w, QGraphicsView):
				w.setVisible(False)
		
		# TODO: connect buttons to functions
		self.btnLoadImage.clicked.connect(self.btnLoadImageClicked)
		self.btnSaveColors.clicked.connect(self.btnSaveColorsClicked)

	def btnLoadImageClicked(self):
		if self.dlgLoadImage.exec_():
			filename = self.dlgLoadImage.selectedFiles()[0]
			self._image = QImage(filename)
			self._imagePixmap = QGraphicsPixmapItem(QPixmap(self._image), None, self._imageScene)
			self.grObjectImage.setScene(self._imageScene)
			self._imagePixmap.mousePressEvent = self.getRGBFromPixel

	def getRGBFromPixel(self, event):
		if len(self._detected_colors) < self._MAX_COLORS:
			pixel_coords = QPoint(event.pos().x(), event.pos().y())
			self._detected_colors.append(QColor.fromRgb(self._image.pixel(pixel_coords)))
			if self._detected_colors[len(self._detected_colors)-1].isValid():
				print "Selected color: RED=" + str(self._detected_colors[len(self._detected_colors)-1].red()) + ", GREEN=" + str(self._detected_colors[len(self._detected_colors)-1].green()) + ", BLUE=" + str(self._detected_colors[len(self._detected_colors)-1].blue())
				widgets = (self.gridLayoutColors.itemAt(i).widget() for i in range(self.gridLayoutColors.count())) 
				for w in widgets:
					if isinstance(w, QGraphicsView) and not w.isVisible():
						w.setVisible(True)
						w.setStyleSheet("background-color: rbg(" + str(self._detected_colors[len(self._detected_colors)-1].red()) + ", " + str(self._detected_colors[len(self._detected_colors)-1].green()) + ", " + str(self._detected_colors[len(self._detected_colors)-1].blue()) + ")")
						break
			else:
				print "Invalid color selected!"
		
	def btnSaveColorsClicked(self):
		if self.dlgSaveColors.exec_():
			# open new color file
			color_file = open(self.dlgSaveColors.selectedFiles()[0], "w")
			# get max and min HSV thresholds and pass them to main application window
			hsv_threshold_max = (0,0,0)
			hsv_threshold_min = (255,255,255)
			for color in self._detected_colors:
				hsv = color.getHsv()
				hsv_threshold_max = (max(hsv_threshold_max[0], hsv[0]), max(hsv_threshold_max[1], hsv[1]), max(hsv_threshold_max[2], hsv[2]))
				hsv_threshold_min = (min(hsv_threshold_min[0], hsv[0]), min(hsv_threshold_min[1], hsv[1]), min(hsv_threshold_min[2], hsv[2]))
			color_file.write("[hsv_max]\n")
			color_file.write(str(hsv_threshold_max).strip("()").replace(" ", "") + "\n")
			color_file.write("[hsv_min]\n")
			color_file.write(str(hsv_threshold_min).strip("()").replace(" ", "") + "\n")
			color_file.close()
			#self._main_window_ref.updateColors(hsv_threshold_max, hsv_threshold_min)

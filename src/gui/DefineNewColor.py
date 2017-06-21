from PyQt4 import uic
from PyQt4.QtCore import QPoint, Qt
from PyQt4.QtGui import QColor, QSizePolicy, QPainter, QWidget, QFileDialog, QGraphicsScene, QGraphicsView, QImage, QBrush, QPixmap, QGraphicsPixmapItem


class UIDefineNewColorWidget(QWidget):
	def __init__(self, main_window_ref):
		super(UIDefineNewColorWidget, self).__init__()
		uic.loadUi('ui_files/DefineNewColor.ui', self)
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
                self.dlgSaveColors.setFilter("Color files (*.clr)")
                self.dlgSaveColors.setAcceptMode(QFileDialog.AcceptSave)

		# initialize graphics view objects and color array
		self.btnClearColorsClicked()
	
		# connect buttons to functions
		self.btnLoadImage.clicked.connect(self.btnLoadImageClicked)
		self.btnSaveColors.clicked.connect(self.btnSaveColorsClicked)
		self.btnClearColors.clicked.connect(self.btnClearColorsClicked)

	def btnLoadImageClicked(self):
		if self.dlgLoadImage.exec_():
			filename = self.dlgLoadImage.selectedFiles()[0]
			self._image = QImage(filename)
			self._imagePixmap = QGraphicsPixmapItem(QPixmap(self._image), None, self._imageScene)
			self.grObjectImage.setScene(self._imageScene)
        		self.grObjectImage.fitInView(self._imageScene.sceneRect(), Qt.KeepAspectRatio)
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
						w.setStyleSheet("background-color: rgb(" + str(self._detected_colors[len(self._detected_colors)-1].red()) + ", " + str(self._detected_colors[len(self._detected_colors)-1].green()) + ", " + str(self._detected_colors[len(self._detected_colors)-1].blue()) + ")")
						break
			else:
				print "Invalid color selected!"
		
	def btnSaveColorsClicked(self):
		if self.dlgSaveColors.exec_():
			# open new color file
			filename = self.dlgSaveColors.selectedFiles()[0]
			if not filename.endsWith(".clr"):
				filename += ".clr"
			color_file = open(filename, "w")

			# save selected colors to file
			for color in self._detected_colors:
				color_file.write(str((color.getRgb())[:-1]) + "\n")
			color_file.close()
			self._main_window_ref.updateColorFileName(filename)

	def btnClearColorsClicked(self):
		# initialize image graphics view
		self._imageScene = QGraphicsScene()
		self._detected_colors = []
		self._detected_colors_index = -1

		# make QGraphicsView objects invisible on startup
		widgets = (self.gridLayoutColors.itemAt(i).widget() for i in range(self.gridLayoutColors.count()))
                for w in widgets:
			if isinstance(w, QGraphicsView):
				w.setVisible(False)

from PyQt4 import uic
from PyQt4.QtCore import QPoint
from PyQt4.QtGui import QColor, QSizePolicy, QWidget, QFileDialog, QGraphicsScene, QGraphicsView, QImage, QPixmap, QGraphicsPixmapItem

class UILoadCustomObjectWidget(QWidget):
	def __init__(self, main_window_ref):
		super(UILoadCustomObjectWidget, self).__init__()
		uic.loadUi('QTRehaZenterGUI_LoadCustomObject.ui', self)
		self._main_window_ref = main_window_ref

		# initialize image selection dialog
		self.dlgLoadImage = QFileDialog()
		self.dlgLoadImage.setFileMode(QFileDialog.ExistingFile)
		self.dlgLoadImage.setFilter("Image files (*.png *.jpeg *.jpg *.bmp)")
		self.dlgLoadImage.setAcceptMode(QFileDialog.AcceptOpen)

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

	def btnLoadImageClicked(self):
		if self.dlgLoadImage.exec_():
			filename = self.dlgLoadImage.selectedFiles()[0]
			self._image = QImage(filename)
			self._imagePixmap = QGraphicsPixmapItem(QPixmap(self._image), None, self._imageScene)
			self.grObjectImage.setScene(self._imageScene)
			self._imagePixmap.mousePressEvent = self.getRGBFromPixel

	def getRGBFromPixel(self, event):
		pixel_coords = QPoint(event.pos().x(), event.pos().y())
		self._detected_colors.append(QColor.fromRgb(self._image.pixel(pixel_coords)))
		self._detected_colors_index += 1
		if self._detected_colors[self._detected_colors_index].isValid():
			print "Selected color: RED=" + str(self._detected_colors[self._detected_colors_index].red()) + ", GREEN=" + str(self._detected_colors[self._detected_colors_index].green()) + ", BLUE=" + str(self._detected_colors[self._detected_colors_index].blue())
			pixel_color_hsv = color.getHsv()
			hsv_threshold_max = (0,0,0)
			hsv_threshold_min = (0,0,0)
			widgets = (self.gridLayoutColors.itemAt(i).widget() for i in range(self.gridLayoutColors.count())) 
			for w in widgets:
				if isinstance(w, QGraphicsView) and not w.isVisible():
					w.setBackgroundBrush(color)
					w.setForegroundBrush(color)
					w.update()
					w.setVisible(True)
					break
			
		else:
			print "Invalid color selected!"
		

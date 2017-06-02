from PyQt4 import uic
from PyQt4.QtCore import QPoint
from PyQt4.QtGui import QColor, QWidget, QFileDialog, QGraphicsScene, QImage, QPixmap, QGraphicsPixmapItem

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
		color = QColor.fromRgb(self._image.pixel(pixel_coords))
		if color.isValid():
			print "Selected color: RED=" + str(color.red()) + ", GREEN=" + str(color.green()) + ", BLUE=" + str(color.blue())
		else:
			print "Invalid color selected!"
		

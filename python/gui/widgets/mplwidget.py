from __future__ import unicode_literals
from PySide2 import QtWidgets
#from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas
#from matplotlib.figure import Figure


#from abc import ABC#, abstractmethod

# Matplotlib canvas class to create figure
class MplCanvas(Canvas):
    def __init__(self, i_oFig):
        self.m_oFig = i_oFig
        #self.ax = self.m_oFig.add_subplot(position)
        Canvas.__init__(self, self.m_oFig)
        Canvas.setSizePolicy(self, QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        Canvas.updateGeometry(self)

# Matplotlib widget
class MplWidget(QtWidgets.QWidget):
    def __init__(self, i_oFig, parent=None):
        print("new MplWidget", parent)
        QtWidgets.QWidget.__init__(self, parent)   # Inherit from QWidget
        self.canvas = MplCanvas(i_oFig)                  # Create canvas object
        self.vbl = QtWidgets.QVBoxLayout()         # Set box for plotting
        self.vbl.addWidget(self.canvas)
        self.setLayout(self.vbl)

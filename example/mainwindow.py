from PySide2.QtWidgets import QMainWindow

import mainwindow_ui as mainui

class FrenetMainWindow(QMainWindow, mainui.Ui_MainWindow):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.pb_set_lat.clicked.connect(self.plot_data)

    def plot_data(self):
        x=range(0, 10)
        y=range(0, 20, 2)
        self.plotWidget.canvas.ax.plot(x, y)
        self.plotWidget.canvas.draw()
        self.plotWidget2.canvas.ax.plot(x, y)
        self.plotWidget2.canvas.draw()

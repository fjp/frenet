from PySide2.QtWidgets import QMainWindow
from ui import frenet_main_window_ui as fmw_ui

class FrenetMainWindow(QMainWindow, fmw_ui.Ui_MainWindow):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.pb_set_lat.clicked.connect(self.plot_data)
        #self.setCentralWidget(self.centralwidget)
        self.dynamics_widget.setMinimumWidth(800)

    def plot_data(self):
        x=range(0, 10)
        y=range(0, 20, 2)
        #self.centralwidget.ax_s.plot(x, y)
        #self.centralwidget.canvas.draw()
        self.dynamics_widget.ax_s.plot(x, y)
        self.dynamics_widget.canvas.draw()
        self.path_widget.ax_path.plot(x, y)
        self.path_widget.canvas.draw()

from gui import frenet_main_window as mainwindow
from src import frenet_optimal_trajectory as frenet

class cController():
    def __init__(self):
        self.m_oGUI = mainwindow.FrenetMainWindow()
        self.m_oFrenet = frenet


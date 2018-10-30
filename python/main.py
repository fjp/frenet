import sys
import os
from PySide2.QtWidgets import QApplication
#from PySide2.QtCore import QFile
print(sys.path)
sys.path.insert(0, os.getcwd() + "/gui")
sys.path.insert(0, os.getcwd() + "/src")
print(sys.path)
from gui import frenet_main_window as fmw
import controller


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # file = QFile("mainwindow.ui")
    #file.open(QFile.ReadOnly)

    #loader = QUiLoader()
    #window = loader.load(file)
    print("Create Frenet Main Window")
    oFrenetMainWindow = fmw.FrenetMainWindow()

    oFrenetMainWindow.show()

    sys.exit(app.exec_())

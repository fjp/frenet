import sys
from PySide2.QtWidgets import QApplication
#from PySide2.QtCore import QFile
import frenet_main_window as fmw

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

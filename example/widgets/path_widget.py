import widgets.mplwidget as mpw
from matplotlib.figure import Figure

# Matplotlib widget
class PathWidget(mpw.MplWidget):
    def __init__(self, parent=None):
        print("Create PathWidget", parent)
        self.oFig = Figure()
        self.ax_path = self.oFig.add_subplot(111)
        super(PathWidget, self).__init__(self.oFig, parent)




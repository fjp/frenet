import widgets.mplwidget as mpw
from matplotlib.figure import Figure

# Matplotlib widget
class DynamicsWidget(mpw.MplWidget):
    def __init__(self, parent=None):
        print("create DynamicsWidget", parent)
        self.oFig = Figure()
        nPosition = 421
        self.ax_s = self.oFig.add_subplot(nPosition+0)
        self.ax_d = self.oFig.add_subplot(nPosition+1)
        self.ax_ds = self.oFig.add_subplot(nPosition+2)
        self.ax_dd = self.oFig.add_subplot(nPosition+3)
        self.ax_dds = self.oFig.add_subplot(nPosition+4)
        self.ax_ddd = self.oFig.add_subplot(nPosition+5)
        self.ax_ddds = self.oFig.add_subplot(nPosition+6)
        self.ax_dddd = self.oFig.add_subplot(nPosition+7)
        super(DynamicsWidget, self).__init__(self.oFig, parent)




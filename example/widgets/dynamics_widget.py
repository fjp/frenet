import widgets.mplwidget as mpw


# Matplotlib widget
class MplDynamicsWidget(mpw.MplWidget):
    def __init__(self, parent=None, position=111):
        print("create MplDynamicsWidget", parent)
        super(MplDynamicsWidget, self).__init__(parent, 421)




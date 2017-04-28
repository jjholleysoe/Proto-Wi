from PlotCanvas import *
from PyQt4.QtCore import SIGNAL
import warnings
import matplotlib.cbook
import time
warnings.filterwarnings("ignore",category=matplotlib.cbook.mplDeprecation)


class ResponsePlotCanvas(MplDynamicPlotCanvas):
    """This is a plot canvas for the response signal."""

    def __init__(self, *args, **kwargs):
        MplDynamicPlotCanvas.__init__(self, *args, **kwargs)
        self.Title = "Response Signal (v)"
    	self.setTitle()
        self.data = []
        self.axes.hold(False)
        self.previousTime = 0

    def connectSlot(self, commThread):
        self.connect(commThread, SIGNAL("new_data_received(PyQt_PyObject)"), self.updatePlot)

    def updatePlot(self, responseData):
        # Keep only last 100 values of data for plotting
        if (len(self.data) > 100):
            self.data.pop(0)
        try:
            if responseData.MeasuredAngle:
                print responseData.MeasuredAngle, time.time() - self.previousTime
                self.previousTime = time.time()
                self.data.append(responseData.MeasuredAngle)
        except IndexError:
            return

    def setTitle(self):
        self.axes.set_title(self.Title)

    def update_figure(self):
        self.axes.plot(self.data, 'b')
        self.setTitle()
        self.draw()

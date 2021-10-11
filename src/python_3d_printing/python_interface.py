# -*- coding: utf-8 -*-
"""
Created on Mon Oct 11 10:11:24 2021

@author: baciu
"""
import sys
import os
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import functools
import numpy as np
import random as rd
import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.figure import Figure
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import time
import threading
from matplotlib.widgets import SpanSelector
from process_serial_frames import SerialFrame
from stl_tools import numpy2stl
from scipy.ndimage import gaussian_filter
from pylab import imread
import subprocess

def setCustomSize(x, width, height):
    sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(0)
    sizePolicy.setHeightForWidth(x.sizePolicy().hasHeightForWidth())
    x.setSizePolicy(sizePolicy)
    x.setMinimumSize(QSize(width, height))
    x.setMaximumSize(QSize(width, height))

''''''

class CustomMainWindow(QMainWindow):

    def __init__(self, COM_port):

        super(CustomMainWindow, self).__init__()
        self.com = COM_port
        
        # Define the geometry of the main window
        self.setGeometry(300, 300, 1000, 450)
        self.setWindowTitle("PPG data aquisition")
        self.State = False
        # Create FRAME_A
        self.FRAME_A = QFrame(self)
        self.FRAME_A.setStyleSheet("QWidget { background-color: %s }" % QColor(255,255,255,255).name())
        self.LAYOUT_A = QGridLayout()
        self.LAYOUT_A.setSpacing(20)
        self.FRAME_A.setLayout(self.LAYOUT_A)
        self.setCentralWidget(self.FRAME_A)

        # Place the zoom button
        self.stopBtn = QPushButton(text = 'Stop data aquisition')
        setCustomSize(self.stopBtn, 100, 50)
        self.stopBtn.setStyleSheet('QPushButton {background-color: lightgrey; color: black;}')
        self.stopBtn.setCheckable(True)
        
        self.convertBtn = QPushButton(text = 'Convert and print')
        self.convertBtn.setStyleSheet('QPushButton {background-color: #39B3C4; color: black;}')
        setCustomSize(self.convertBtn, 100, 50)
        
        self.stopBtn.clicked.connect(self.stopBtnAction)
        self.convertBtn.clicked.connect(self.convert_and_print)
        
        self.LAYOUT_A.addWidget(self.stopBtn, *(1,0))
        self.LAYOUT_A.addWidget(self.convertBtn, *(1,1))
        # Place the matplotlib figure
        self.myFig = CustomFigCanvas(show_tail = True)
        self.LAYOUT_A.addWidget(self.myFig, *(1,2))
        self.myFig2 = CustomFigCanvas(show_tail = False)
        self.myFig2.ax1.axis('off')
        
        self.LAYOUT_A.addWidget(self.myFig2, *(2,2))
        
        self.span = SpanSelector(
            self.myFig.ax1,
            self.onselect,
            "horizontal",
            useblit=True,
            rectprops=dict(alpha=0.5, facecolor="red"))   
        # Add the callbackfunc to ..
        myDataLoop = threading.Thread(name = 'myDataLoop', target = dataSendLoop, args = (self.addData_callbackFunc,))
        myDataLoop.start()

        self.show()
        
    def closeEvent(self, event):
        super(CustomMainWindow, self).closeEvent(event)
        self.com.close()
        self.close()
    ''''''
    def onselect(self, min_value, max_value):
        
        x, y = self.myFig.getIndex(min_value, max_value)
   
        self.myFig2.plotSelection(x,y)
        
    def convert_and_print(self):
        
        self.myFig2.print_figure('out/ppg_selection_out.png', pad_inches=0)
        
        A = 256 * imread("out/ppg_selection_out.png")
        A = A[:, :, 2] + 1.0*A[:,:, 0] # Compose RGBA channels to give depth
        A = gaussian_filter(A, 1)  # smoothing
        numpy2stl(A, "out/ppg_selection_out.stl", scale=0.05, mask_val=5., solid=False)
        ini_file_loc = os.path.join(os.getcwd(),'in', 'PrusaSlicer_config_bundle.ini')
        stl_file_loc = os.path.join(os.getcwd(),'out', 'ppg_selection_out.stl')
     
        command = "prusa-slicer-console.exe -printer-technology FFF -center 125,105 -g {} -load {}".format(stl_file_loc,ini_file_loc)
        print(command)
        subprocess.run(command, shell=True)
    
    def stopBtnAction(self):
        if self.stopBtn.isChecked():
  
            # setting background color to light-blue
            self.stopBtn.setStyleSheet("background-color : lightblue")
            self.State = True
  
        # if it is unchecked
        else:
  
            # set background color back to light-grey
            self.stopBtn.setStyleSheet("background-color : lightgrey")
            self.State = False
            
        self.myFig.stopBtn(self.State)

    ''''''

    def addData_callbackFunc(self, value):
        # print("Add data: " + str(value))
        self.myFig.addData(value)



''' End Class '''


class CustomFigCanvas(FigureCanvas, TimedAnimation):

    def __init__(self, show_tail):
        self.addedData = []
        self.stopDataAq = False
        self.show_tail = show_tail
    
        # The data
        self.xlim = 100
        self.n = np.linspace(0, self.xlim - 1, self.xlim)
        a = []
        b = []
        a.append(2.0)
        a.append(4.0)
        a.append(2.0)
        b.append(4.0)
        b.append(3.0)
        b.append(4.0)
        self.y = (self.n * 0.0) + 50

        # The window
        self.fig = Figure(figsize=(5,5), dpi=100)
        self.ax1 = self.fig.add_subplot(111)


        # self.ax1 settings
        #self.ax1.set_xlabel('Time')
        self.ax1.get_xaxis().set_visible(False)
        self.ax1.get_yaxis().set_visible(False)
        
        self.ax1.set_ylabel('Raw data')
        if self.show_tail == False:
            self.line1 = Line2D([], [], color='blue', linewidth = 10)
        else:
            self.line1 = Line2D([], [], color='blue') 
        self.ax1.add_line(self.line1)
        if self.show_tail == True:
            self.line1_tail = Line2D([], [], color='red', linewidth=2)
            self.line1_head = Line2D([], [], color='red', marker='o', markeredgecolor='r')
            self.ax1.add_line(self.line1_tail)
            self.ax1.add_line(self.line1_head)
        self.ax1.set_xlim(0, self.xlim - 1)
        self.ax1.set_ylim(-10, 100)


        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval = 50, blit = True)

    def new_frame_seq(self):
        return iter(range(self.n.size))

    def _init_draw(self):
        if self.show_tail == True:
            lines = [self.line1, self.line1_tail, self.line1_head]
        else:
            lines = [self.line1]
            
        for l in lines:
            l.set_data([], [])

    def addData(self, value):
        if self.stopDataAq == False:
            self.addedData.append(value)

    def stopBtn(self, value):
        self.stopDataAq = value
    
    def getIndex(self, xmin, xmax):
        indmin, indmax = np.searchsorted(self.n, (xmin, xmax))
        indmax = min(len(self.n) - 1, indmax)
        thisx = self.n[indmin:indmax]
        thisy = self.y[indmin:indmax]
        
        return thisx, thisy
    
    def plotSelection(self, thisx, thisy):
        self.n = thisx
        self.y = thisy
        
        self.ax1.set_xlim(thisx[0], thisx[-1])
        self.ax1.set_ylim(thisy.min() - 20, thisy.max() + 20)
        self._draw_frame(thisx)
        self.fig.canvas.draw_idle()
        
    def _step(self, *args):
        # Extends the _step() method for the TimedAnimation class.
        try:
            TimedAnimation._step(self, *args)
        except Exception as e:
            self.abc += 1
            print(str(self.abc))
            TimedAnimation._stop(self)
            pass

    def _draw_frame(self, framedata):
        margin = 0
        while(len(self.addedData) > 0):
            self.y = np.roll(self.y, -1)
            self.y[-1] = self.addedData[0]
            del(self.addedData[0])


        self.line1.set_data(self.n[ 0 : self.n.size - margin ], self.y[ 0 : self.n.size - margin ])
        if self.show_tail == True:
             
            self.ax1.set_ylim(self.y.min() - 50, self.y.max() + 50)
            self.line1_tail.set_data(np.append(self.n[-10:-1 - margin], self.n[-1 - margin]), np.append(self.y[-10:-1 - margin], self.y[-1 - margin]))
            self.line1_head.set_data(self.n[-1 - margin], self.y[-1 - margin])
            self._drawn_artists = [self.line1, self.line1_tail, self.line1_head]
        else:
            self._drawn_artists = [self.line1]



''' End Class '''


# You need to setup a signal slot mechanism, to
# send data to your GUI in a thread-safe way.
# Believe me, if you don't do this right, things
# go very very wrong..
class Communicate(QObject):
    data_signal = pyqtSignal(float)

''' End Class '''

serialHandler = SerialFrame('COM4', 115200)

def dataSendLoop(addData_callbackFunc):
    # Setup the signal-slot mechanism.

    mySrc = Communicate()
    mySrc.data_signal.connect(addData_callbackFunc)

    i = 0

    while(True):
        try:
            serialHandler.readAndProcessFrame()
        except:
            pass
        
        if(serialHandler.ir_buffer_updated):
                for i in range(0, len(serialHandler.ir_buffer)-1):
                    time.sleep(0.1)
                    mySrc.data_signal.emit(serialHandler.ir_buffer[i]) # <- Here you emit a signal!   

    ###
###




if __name__== '__main__':
    
    app = QApplication(sys.argv)
    QApplication.setStyle(QStyleFactory.create('Plastique'))
    myGUI = CustomMainWindow(serialHandler.serialHandler)
    sys.exit(app.exec_())
    quit()
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 11 10:11:24 2021

@author: baciu
"""
from PyQt5.QtWidgets import QApplication, QDialog, QVBoxLayout, QLabel
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from matplotlib.figure import Figure
from matplotlib.animation import TimedAnimation
from matplotlib.lines import Line2D
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from process_serial_frames import SerialFrame


import numpy as np
import matplotlib
import time
import threading

import os, os.path, inspect
import sys


matplotlib.use("Qt5Agg")


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

        

        if '__file__' not in locals():
            __file__ = inspect.getframeinfo(inspect.currentframe())[0]
        
        self.dir_path = os.path.dirname(os.path.abspath(__file__))
       
        
        super(CustomMainWindow, self).__init__()
        self.com = COM_port
        # Define the geometry of the main window
        self.setGeometry(300, 300, 1500, 500)
        self.setWindowTitle("PPG VUB")
        self.State = False
        # Create FRAME_A
        self.FRAME_A = QFrame(self)
        self.FRAME_A.setStyleSheet("QWidget { background-color: %s }"
                                   % QColor(255, 255, 255, 255).name())
        self.LAYOUT_A = QGridLayout()
        self.LAYOUT_A.setSpacing(20)
        self.FRAME_A.setLayout(self.LAYOUT_A)
        self.setCentralWidget(self.FRAME_A)

        # Create the stop button
        self.stopBtn = QPushButton(text='Clear plot')
        setCustomSize(self.stopBtn, 0, 0)
        self.stopBtn.setStyleSheet('QPushButton {background-color: lightgrey; color: black;}')
        self.stopBtn.setCheckable(True)

        # Create the convert button
        self.convertBtn = QPushButton(text='Convert and print')
        self.convertBtn.setStyleSheet('QPushButton {background-color: #39B3C4; color: black;}')
        setCustomSize(self.convertBtn, 0, 0)
        self.labelImage = QLabel(self)
        self.pixmap = QPixmap("vub.jpg")
        self.labelImage.setPixmap(self.pixmap)
        setCustomSize(self.labelImage, 274,70)
        # Set an action for each button
        self.stopBtn.clicked.connect(self.stopBtnAction)


        # Place the buttons on LAYOUT_A
        self.LAYOUT_A.addWidget(self.labelImage, *(1, 0))
      
        # Place the matplotlib figures
        self.myFig = CustomFigCanvas(show_tail=True)
        self.LAYOUT_A.addWidget(self.myFig, *(1, 2))

        
    



        self.counter = 0
        # Add the callbackfunc to read the serial buffer
        myDataLoop = threading.Thread(name='myDataLoop', target=dataSendLoop,
                                      args=(self.addData_callbackFunc,))
        myDataLoop.start()

        self.show()

    def closeEvent(self, event):
        super(CustomMainWindow, self).closeEvent(event)
        # Close serial connection
        self.com.close()
        # Close interface
        self.close()

    ''''''
    
    def stopBtnAction(self):
        if self.stopBtn.isChecked():
            # setting background color to light-blue
            self.stopBtn.setStyleSheet("background-color : lightblue")
            self.State = True
        else:  # if it is unchecked
            # set background color back to light-grey
            self.stopBtn.setStyleSheet("background-color : lightgrey")
            self.State = False

        # Call the stopBtn function for the PPG recording graph
        self.myFig.stopBtn(self.State)

    # Callback function for adding data to the buffer which will be displayed on
    # the PPG recording graph
    def addData_callbackFunc(self, value):
        self.myFig.addData(value)

    ''''''

''' End Class '''


class CustomFigCanvas(FigureCanvas, TimedAnimation):

    def __init__(self, show_tail):
        self.addedData = []
        self.stopDataAq = False
        self.show_tail = show_tail

        # The default data
        self.xlim = 300
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
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax1 = self.fig.add_subplot(111)

        # self.ax1.set_xlabel('Time')
        # self.ax1.set_ylabel('Raw data')
        # Disable axis since the purpose is to have a 3D print
        self.fig.suptitle("PPG Signal aquisition", fontsize = 16)
        self.ax1.get_xaxis().set_visible(True)
        self.ax1.get_yaxis().set_visible(True)
        self.ax1.get_xaxis().set_ticks([])
        # Check if the linewidth should be increased
        if self.show_tail is False:
            self.line1 = Line2D([], [], color='blue', linewidth=10)
        else:
            self.line1 = Line2D([], [], color='blue')

        self.ax1.add_line(self.line1)

        # Check if the new datapoints should be displayed as a colorful tail
        if self.show_tail is True:
            self.line1_tail = Line2D([], [], color='red', linewidth=2)
            self.line1_head = Line2D([], [], color='red', marker='o', markeredgecolor='r')
            self.ax1.add_line(self.line1_tail)
            self.ax1.add_line(self.line1_head)

        self.ax1.set_xlim(0, self.xlim - 1)
        self.ax1.set_xlabel('Samples', fontsize=16)
        self.ax1.set_ylabel('Amplitude', fontsize=16)
        #self.ax1.set_ylim(-1500, 1500)

        FigureCanvas.__init__(self, self.fig)
        TimedAnimation.__init__(self, self.fig, interval=50, blit=True)

    def new_frame_seq(self):
        return iter(range(self.n.size))

    # Init the graph with default data
    def _init_draw(self):
        if self.show_tail is True:
            lines = [self.line1, self.line1_tail, self.line1_head]
        else:
            lines = [self.line1]

        for l in lines:
            l.set_data([], [])

    # Add data to the plot buffer
    def addData(self, value):
        if self.stopDataAq is False:
            self.addedData.append(value)
        else:
            self.addedData = []
    # Stop updating the plot buffer
    def stopBtn(self, value):
        self.fig.close()
        

    # Get the graph values for the span selector
    def getIndex(self, xmin, xmax):
        indmin, indmax = np.searchsorted(self.n, (xmin, xmax))
        indmax = min(len(self.n) - 1, indmax)
        thisx = self.n[indmin:indmax]
        thisy = self.y[indmin:indmax]

        return thisx, thisy



    def _step(self, *args):
        # Extends the _step() method for the TimedAnimation class.
        try:
            TimedAnimation._step(self, *args)
        except Exception as e:
            self.abc += 1
            print(str(self.abc))
            TimedAnimation._stop(self)
            pass

    # Draw frame as long as the addedData contains new data
    def _draw_frame(self, framedata):
        margin = 1
        while(len(self.addedData) > 0):
            self.y = np.roll(self.y, -1)
            
            self.y[-1] = self.addedData[0]
            del(self.addedData[0])

        self.line1.set_data(self.n[0:self.n.size - margin], self.y[0:self.n.size - margin])
        if self.show_tail is True:
            if self.y.max() > 30000:
                self.ax1.set_ylim(self.y.min() - 1500, 1000)
            else:
                self.ax1.set_ylim(self.y.min() - 1500, self.y.max() + 1500)
            self.line1_tail.set_data(
                                    np.append(self.n[-10:-1 - margin], self.n[-1 - margin]),
                                    np.append(self.y[-10:-1 - margin], self.y[-1 - margin])
                                    )
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



# Target function for read and plot data thread
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

        if(serialHandler.tissue_detected):
                for i in range(0, len(serialHandler.ir_buffer)-1):
                    time.sleep(0.06)
                    # Emit a signal!
                    mySrc.data_signal.emit(serialHandler.ir_buffer[i])

if __name__ == '__main__':

    
    if '__file__' not in locals():
        __file__ = inspect.getframeinfo(inspect.currentframe())[0]
        
    ini_file_path = os.path.dirname(os.path.abspath(__file__)) + "/serial.ini"
    ini_file = open(ini_file_path, "r")
    com_port = ini_file.readline().rstrip()
    baud = int(ini_file.readline().rstrip())
    
    serialHandler = SerialFrame(com_port, baud)

    app = QApplication(sys.argv)
    QApplication.setStyle(QStyleFactory.create('Plastique'))
    myGUI = CustomMainWindow(serialHandler.serialHandler)
    sys.exit(app.exec_())
    quit()

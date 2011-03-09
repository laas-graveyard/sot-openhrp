#!/usr/bin/python

import csv
from dynamic_graph.sot.se3 import R3
import matplotlib.pyplot as pl

rawData = []
data = []

class Log(object):
    """
    Read and plot log files from OpenHRP
    """
    config = []
    """
    Configurations: vectors of dimension 40
    """
    torque = []
    """
    Torques: vectors of dimension 40
    """
    forceRa = []
    """
    Force at right ankle
    """
    momentRa = []
    """
    Moment at right Ankle
    """
    forceLa = []
    """
    Force at left ankle
    """
    momentLa = []
    """
    Moment at left Ankle
    """
    forceRw = []
    """
    Force at right wrist
    """
    momentRw = []
    """
    Moment at right wrist
    """
    forceLw = []
    """
    Force at left wrist
    """
    momentLw = []
    """
    Moment at left wrist
    """
    zmpRf = []
    """
    Center of pressure of right foot expressed in foot frame
    """
    zmpLf = []
    """
    Center of pressure of left foot expressed in foot frame
    """
    leftAnkle = R3((0.0, 0.0, 0.105))
    """
    Position of left ankle in foot frame: default value for HRP2
    """
    rightAnkle = R3((0.0, 0.0, 0.105))
    """
    Position of right ankle in foot frame: default value for HRP2
    """
    directory = '/opt/grx3.0/HRP2LAAS/bin'
    """
    Directory where to find log files
    """

    def __init__(self, filename = None, robot = None):
        """
        Construct instance with a robot to get geometry of feet
        """
        if filename:
            self.filename = filename
        else:
            self.filename = self.directory + '/sim-astate.log'

        if robot:
            self.leftAnkle = robot.dynamic.getAnklePositionInFootFrame()
            self.rightAnkle = R3(self.leftAnkle)
            self.rightAnkle[1] *= -1.
        self.read()

    def read(self):
        filename = self.filename
        with open(filename, 'r') as f:
            astateReader = csv.reader(f, delimiter=' ')
            for row, i in zip(astateReader, range(10000)):
                while '' in row:
                    row.remove('')
                try:
                    r = map(float, row)
                    data.append(r)
                except:
                    print row

        for r in data:
            self.config.append(r[:40])
            self.torque.append(r[40:80])
            self.forceRa.append(R3(tuple(r[80:83])))
            self.momentRa.append(R3(tuple(r[83:86])))
            self.forceLa.append(R3(tuple(r[86:89])))
            self.momentLa.append(R3(tuple(r[89:92])))
            self.forceRw.append(R3(tuple(r[92:95])))
            self.momentRw.append(R3(tuple(r[95:98])))
            self.forceLw.append(R3(tuple(r[98:101])))
            self.momentLw.append(R3(tuple(r[101:104])))

        # Compute centers of pressure
        for R, Ma in zip(self.forceRa, self.momentRa):
            Mo = Ma + self.leftAnkle.crossprod(R)
            if R[2] > 10:
                self.zmpRf.append(R3((Mo[0]/R[2], -Mo[1]/R[2], 0.)))
            else:
                self.zmpRf.append(R3((0.,0.,0.,)))

        for R, Ma in zip(self.forceLa, self.momentLa):
            Mo = Ma + self.rightAnkle.crossprod(R)
            if R[2] > 10:
                self.zmpLf.append(R3((Mo[0]/R[2], -Mo[1]/R[2], 0.)))
            else:
                self.zmpLf.append(R3((0.,0.,0.,)))

    def plot(self):
        fig1 = pl.figure()
        ax1 = fig1.add_subplot(211)
        ax2 = fig1.add_subplot(212)
        zmpRf = []
        zmpLf = []
        y0 = []
        y1 = []
        for (F0, F1) in zip(self.forceRa,self.forceLa):
            y0.append(F0[2])
            y1.append(F1[2])
        time = map(lambda x:.005*x, range(len(y0)))
        ax1.plot(time, y0)
        ax1.plot(time, y1)
        for (z0, z1) in zip(self.zmpRf, self.zmpLf):
            zmpRf.append(z0[1])
            zmpLf.append(z1[1])
        ax2.plot(time, zmpRf)
        ax2.plot(time, zmpLf)

        ax1.legend(('force right ankle', 'force left ankle'))
        ax2.legend(('zmp right foot', 'zmp left foot'))

        pl.show()

if __name__ == '__main__':
    import sys

    filename = None
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    l = Log(filename = filename)
    l.plot()

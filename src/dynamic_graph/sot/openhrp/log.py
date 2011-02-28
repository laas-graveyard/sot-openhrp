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
    force0 = []
    """
    Force at left ankle
    """
    moment0 = []
    """
    Moment at left Ankle
    """
    force1 = []
    """
    Force at right ankle
    """
    moment1 = []
    """
    Moment at right Ankle
    """
    force2 = []
    """
    Force at left wrist
    """
    moment2 = []
    """
    Moment at left wrist
    """
    force3 = []
    """
    Force at right wrist
    """
    moment3 = []
    """
    Moment at right wrist
    """
    zmp0 = []
    """
    Center of pressure of left foot expressed in foot frame
    """
    zmp1 = []
    """
    Center of pressure of right foot expressed in foot frame
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

    def __init__(self, dir = None, robot = None):
        """
        Construct instance with a robot to get geometry of feet
        """
        if dir:
            self.directory = dir
        if robot:
            self.leftAnkle = robot.dynamic.getAnklePositionInFootFrame()
            self.rightAnkle = R3(self.leftAnkle)
            self.rightAnkle[1] *= -1.
        self.read()

    def read(self):
        filename = self.directory + '/sim-astate.log'
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
            self.force0.append(R3(tuple(r[80:83])))
            self.moment0.append(R3(tuple(r[83:86])))
            self.force1.append(R3(tuple(r[86:89])))
            self.moment1.append(R3(tuple(r[89:92])))
            self.force2.append(R3(tuple(r[92:95])))
            self.moment2.append(R3(tuple(r[95:98])))

        # Compute centers of pressure
        for R, Ma in zip(self.force0, self.moment0):
            Mo = Ma + self.leftAnkle.crossprod(R)
            if R[2] > 10:
                self.zmp0.append(R3((Mo[0]/R[2], -Mo[1]/R[2], 0.)))
            else:
                self.zmp0.append(R3((0.,0.,0.,)))

        for R, Ma in zip(self.force1, self.moment1):
            Mo = Ma + self.rightAnkle.crossprod(R)
            if R[2] > 10:
                self.zmp1.append(R3((Mo[0]/R[2], -Mo[1]/R[2], 0.)))
            else:
                self.zmp1.append(R3((0.,0.,0.,)))

    def plot(self):
        fig1 = pl.figure()
        ax11 = fig1.add_subplot(221)
        ax12 = fig1.add_subplot(222)
        ax21 = fig1.add_subplot(223)
        ax22 = fig1.add_subplot(224)
        zmp0 = []
        zmp1 = []
        y0 = []
        y1 = []
        for (F0, F1) in zip(self.force0,self.force1):
            y0.append(F0[2])
            y1.append(F1[2])
        time = map(lambda x:.005*x, range(len(y0)))
        ax11.plot(time, y0)
        ax12.plot(time, y1)
        for (z0, z1) in zip(self.zmp0, self.zmp1):
            zmp0.append(z0[1])
            zmp1.append(z1[1])
        ax21.plot(time, zmp0)
        ax22.plot(time, zmp1)
        pl.show()

if __name__ == '__main__':
    l = Log()
    l.plot()


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
    acceleration = []
    """
    Joint acceleration
    """
    velocity = []
    """
    Joint velocity
    """
    A = []
    """
    Vector AX0 AY0 AZ0 as logged in rstate.log
    """
    zmp = []
    """
    Center of pressure as logged in rstate.log
    """
    waistPos = []
    """
    Presumably position of the waist as logged in rstate.log
    """
    waistOrient = []
    """
    Presumably waist orientation as logged in rstate.log
    """
    leftAnkle = R3((0.0, 0.0, 0.105))
    """
    Position of left ankle in foot frame: default value for HRP2
    """
    rightAnkle = R3((0.0, 0.0, 0.105))
    """
    Position of right ankle in foot frame: default value for HRP2
    """
    directory = '/tmp'
    """
    Directory where to find log files
    """
    prefix = "sim"
    """
    Prefix of log filenames
    """

    def __init__(self, directory = None, prefix = None, robot = None):
        """
        Construct instance with a robot to get geometry of feet
        """
        if prefix:
            self.prefix=prefix
        if directory:
            self.directory = directory

        if robot:
            self.leftAnkle = robot.dynamic.getAnklePositionInFootFrame()
            self.rightAnkle = R3(self.leftAnkle)
            self.rightAnkle[1] *= -1.
        self.read_astate()
        self.read_rstate()
        self.read_kf()

    def read_astate(self):
        filename = self.directory + "/" + self.prefix + "-astate.log"
        with open(filename, 'r') as f:
            reader = csv.reader(f, delimiter=' ')
            for row in reader:
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
                self.zmpRf.append(R3((-Mo[1]/R[2], Mo[0]/R[2], 0.)))
            else:
                self.zmpRf.append(R3((0.,0.,0.,)))

        for R, Ma in zip(self.forceLa, self.momentLa):
            Mo = Ma + self.rightAnkle.crossprod(R)
            if R[2] > 10:
                self.zmpLf.append(R3((-Mo[1]/R[2], Mo[0]/R[2], 0.)))
            else:
                self.zmpLf.append(R3((0.,0.,0.,)))

    def read_rstate(self):
        filename = self.directory + "/" + self.prefix + "-rstate.log"
        with open(filename, 'r') as f:
            reader = csv.reader(f, delimiter=' ')
            for row in reader:
                while '' in row:
                    row.remove('')
                try:
                    r = map(float, row)
                    data.append(r)
                except:
                    print row

        for r in data:
            self.acceleration.append(r[:40])
            self.velocity.append(r[40:80])
            self.A.append(r[80:83])
            self.zmp.append(R3(tuple(r[83:86])))
            self.waistPos.append(R3(tuple(r[86:89])))
            self.waistOrient.append(tuple(r[89:92]))

    def read_kf(self):
        pass

    def computeZmpDoubleSupport(self, ra, la):
        """
        Compute double support center of pressure
          la: position of left anlke
          ra: position of right ankle
        """
        self.zmpDoubleSupport = []
        # Compute centers of pressure
        for Rr, Mar, Rl, Mal in zip(self.forceRa, self.momentRa,
                                    self.forceLa, self.momentLa):
            Mo = Mar + ra.crossprod(Rr) + Mal + la.crossprod(Rl)
            R = Rr+Rl
            if R[2] > 10:
                self.zmpDoubleSupport.append(R3((-Mo[1]/R[2], Mo[0]/R[2], 0.)))
            else:
                self.zmpLf.append(R3((0.,0.,0.,)))


    def plot(self):
        fig1 = pl.figure()
        ax1 = fig1.add_subplot(211)
        ax2 = fig1.add_subplot(212)
        zmpRfx = []
        zmpLfx = []
        zmpRfy = []
        zmpLfy = []
        zmpx = []
        zmpy = []
        Mrx = []
        Mry = []
        Mrz = []
        Mlx = []
        Mly = []
        Mlz = []

        Fnr = []
        Fnl = []
        # Compute zmp for double support
        la = R3((0., 0.095, 0.105)) 
        ra = R3((0., -0.095, 0.105))
        self.computeZmpDoubleSupport(ra, la)
        for (F0, F1) in zip(self.forceRa,self.forceLa):
            Fnr.append(F0[2])
            Fnl.append(F1[2])
        for (zRight, zLeft, z) in zip(self.zmpRf, self.zmpLf,
                                      self.zmpDoubleSupport):
            zmpRfx.append(zRight[0])
            zmpRfy.append(zRight[1])
            zmpLfx.append(zLeft[0])
            zmpLfy.append(zLeft[1])
            zmpx.append(z[0])
            zmpy.append(z[1])

        for (Mr, Ml) in zip(self.momentRa, self.momentLa):
            Mrx.append(Mr[0])
            Mry.append(Mr[1])
            Mrz.append(Mr[2])
            Mlx.append(Ml[0])
            Mly.append(Ml[1])
            Mlz.append(Ml[2])

        time = map(lambda x:.005*x, range(len(Fnr)))
        ax1.plot(time, Fnr)
        ax1.plot(time, Fnl)

        ax2.plot(time, zmpRfx)
        ax2.plot(time, zmpLfx)
        ax2.plot(time, zmpRfy)
        ax2.plot(time, zmpLfy)

        ax1.legend(('force right ankle', 'force left ankle'))
        ax2.legend(('x zmp right foot', 'x zmp left foot',
                    'y zmp right foot', 'y zmp left foot'))

        pl.show()

if __name__ == '__main__':
    #
    # Usage: log.py prefix directory
    #    read in directory files
    #      ${prefix}-astate
    #      ${prefix}-rstate
    #      ${prefix}-kf
    # and plot some of values
    import sys

    directory = None
    prefix = None
    if len(sys.argv) > 1:
        prefix = sys.argv[1]
    if len(sys.argv) > 2:
        directory = sys.argv[2]
    l = Log(prefix = prefix, directory = directory)
    l.plot()

from ClosedLoopRobotController import ClosedLoopRobotController
from math import atan2, isnan
from numpy.linalg import norm
from numpy import array, dot

class OneWaypointRobotController(ClosedLoopRobotController):
    overshoot=0 #cm', distance along baseline from waypoint to target point
    stateheading='wptX,wptY,baseX,baseY,targetX,targetY,baselineX,baselineY,togoX,togoY,dot'
    def __init__(self,interface):
        ClosedLoopRobotController.__init__(self, interface)
        self.pos       =array([float('nan'),float('nan')])
        self.base      =array([float('nan'),float('nan')])
        self.wpt       =array([4000.0,4000.0])
        self.tgt       =array([float('nan'),float('nan')])
        self.baseline  =array([float('nan'),float('nan')])
        self.baseDir   =array([float('nan'),float('nan')])
        self.heading   =       float('nan')
        self.cmdHeading=       float('nan')
        self.dotp      =       float('nan')
    def navigate(self):
        self.pos=self.interface.pos
        self.heading=self.interface.heading
    def guide(self):
        if isnan(self.base[0]):
            self.base=self.pos
            #Define the baseline, a vector from the waypoint to the base point
            self.baseline=self.base-self.wpt
            #figure the location of the target point
            self.baseDir=self.baseline/norm(self.baseline)
            self.tgt=self.wpt-self.overshoot*self.baseDir
        self.togo=self.pos-self.wpt #vector of space remaining to go, from waypoint to robot
        self.dotp=dot(self.baseline,self.togo)
        if self.dotp<0:
            self.interface.throttle(0) #hit the brakes        
        self.cmdHeading=atan2((self.tgt[0]-self.pos[0]),(self.tgt[1]-self.pos[1]))
    def state(self):
        return ("%0.6f,%0.6f," % tuple(self.wpt)
               +"%0.6f,%0.6f," % tuple(self.base)
               +"%0.6f,%0.6f," % tuple(self.tgt)
               +"%0.6f,%0.6f," % tuple(self.baseline)
               +"%0.6f,%0.6f," % tuple(self.togo)
               +"%0.6f" % self.dotp)

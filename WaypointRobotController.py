from ClosedLoopRobotController import ClosedLoopRobotController
from Robot import coerceHeadingRad
from math import atan2, isnan
from numpy.linalg import norm
from numpy import array, dot

class WaypointRobotController(ClosedLoopRobotController):
    overshoot=0 #cm', distance along baseline from waypoint to target point
    stateheading='wptX,wptY,baseX,baseY,targetX,targetY,baselineX,baselineY,togoX,togoY,dot,cmdHdg'
    def __init__(self,interface,wpts):
        ClosedLoopRobotController.__init__(self, interface)
        self.pos       =array([float('nan'),float('nan')])
        self.base      =array([float('nan'),float('nan')])
        self.wpts      =wpts
        self.i_wpt     =0
        self.tgt       =array([float('nan'),float('nan')])
        self.baseline  =array([float('nan'),float('nan')])
        self.baseDir   =array([float('nan'),float('nan')])
        self.heading   =       float('nan')
        self.cmdHeading=       float('nan')
        self.dotp      =       float('nan')
    def navigate(self):
        self.pos=self.interface.pos
        self.heading=self.interface.heading
    def recalcWaypoint(self):
        self.base=self.pos
        #Define the baseline, a vector from the waypoint to the base point
        self.baseline=self.base-self.wpts[self.i_wpt]
        #figure the location of the target point
        self.baseDir=self.baseline/norm(self.baseline)
        self.tgt=self.wpts[self.i_wpt]-self.overshoot*self.baseDir
    def guide(self):
        if isnan(self.base[0]):
            self.recalcWaypoint()
        self.cmdHeading=coerceHeadingRad(atan2((self.tgt[0]-self.pos[0]),(self.tgt[1]-self.pos[1])))
        self.togo=self.pos-self.wpts[self.i_wpt] #vector of space remaining to go, from waypoint to robot
        self.dotp=dot(self.baseline,self.togo)
        if self.dotp<0:
            self.i_wpt=self.i_wpt+1   #Go to the next waypoint
            if self.i_wpt==len(self.wpts):
                # If this was the last waypoint, start steering back towards the first one just to have something to do
                self.i_wpt=0
                self.interface.throttle.write(0) #hit the brakes
            self.recalcWaypoint()
    def state(self):
        return ("%0.6f,%0.6f," % tuple(self.wpts[self.i_wpt])
               +"%0.6f,%0.6f," % tuple(self.base)
               +"%0.6f,%0.6f," % tuple(self.tgt)
               +"%0.6f,%0.6f," % tuple(self.baseline)
               +"%0.6f,%0.6f," % tuple(self.togo)
               +"%0.6f," % self.dotp
               +"%0.6f" % self.cmdHeading)

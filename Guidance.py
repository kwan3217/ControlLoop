from Robot import coerceHeadingRad, RobotGuidance
from copy import copy
from math import atan2, isnan
from numpy.linalg import norm
from numpy import array, dot, poly1d, polyfit, zeros, hstack, vstack

class WaypointGuidance(RobotGuidance):
    overshoot=500 #cm', distance along baseline from waypoint to target point
    stateheading='gt,gposX,gposY,wptX,wptY,baseX,baseY,targetX,targetY,baselineX,baselineY,togoX,togoY,dot,'+RobotGuidance.stateheading
    def __init__(self,nav,wpts):
        RobotGuidance.__init__(self, nav)
        self.base      =array([float('nan'),float('nan')])
        self.pos       =array([float('nan'),float('nan')])
        self.wpts      =wpts
        self.i_wpt     =0
        self.tgt       =array([float('nan'),float('nan')])
        self.baseline  =array([float('nan'),float('nan')])
        self.baseDir   =array([float('nan'),float('nan')])
        self.togo      =array([float('nan'),float('nan')])
        self.dotp      =       float('nan')
        self.t         =float('nan')
        self.n         =3 #different from the position size so that we know how to stick two-element vectors into it
        self.tHist=zeros(self.n)      #Time history
        self.pHist=zeros([self.n,2])  #Position history
        self.m         =array([float('nan'),float('nan')]) #fitted speed
        self.b         =array([float('nan'),float('nan')]) #fitted location at t0
        self.a         =array([float('nan'),float('nan')]) #dot
    def recalcWaypoint(self):
        self.base=copy(self.pos)
        #Define the baseline, a vector from the waypoint to the base point
        self.baseline=self.base-self.wpts[self.i_wpt]
        #figure the location of the target point
        self.baseDir=self.baseline/norm(self.baseline)
        self.tgt=self.wpts[self.i_wpt]-self.overshoot*self.baseDir
    def incWaypoint(self):
        self.i_wpt=self.i_wpt+1   #Go to the next waypoint
        if self.i_wpt==len(self.wpts):
            # If this was the last waypoint, start steering back towards the first one just to have something to do
            self.i_wpt=0
            self.go=False #hit the brakes
        self.recalcWaypoint()
    def recordHistory(self):
        #Record the current time
        self.tHist=hstack((self.tHist[1:],self.nav.tPPS))
        self.pos=copy(self.nav.pos)
        self.pHist=vstack((self.pHist[1:,:],copy(self.pos)))
    def guide(self):
        self.t=self.nav.t
        if self.nav.hasNewPos:
            self.nav.hasNewPos=False
            self.nav.hasPPS=False
            self.recordHistory()
            if isnan(self.base[0]):
                self.recalcWaypoint()
            self.togo=self.pos-self.wpts[self.i_wpt] #vector of space remaining to go, from waypoint to robot
            self.dotp=dot(self.baseline,self.togo)
            if self.dotp<0:
                self.incWaypoint()
            #Calculate the heading AFTER we update the waypoint. This should fix issue #6
            self.cmdHeading=coerceHeadingRad(atan2((self.tgt[0]-self.pos[0]),(self.tgt[1]-self.pos[1])))
        if self.t>self.tPredictWpt:
            self.incWaypoint()
    def state(self):
        return ("%0.6f," % self.t
               +"%0.6f,%0.6f," % tuple(self.pos)
               +"%0.6f,%0.6f," % tuple(self.wpts[self.i_wpt])
               +"%0.6f,%0.6f," % tuple(self.base)
               +"%0.6f,%0.6f," % tuple(self.tgt)
               +"%0.6f,%0.6f," % tuple(self.baseline)
               +"%0.6f,%0.6f," % tuple(self.togo)
               +"%0.6f," % self.dotp
               +RobotGuidance.state(self))

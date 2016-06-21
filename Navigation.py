from Robot import RobotNavigator
from math import isnan
from copy import copy
from numpy import array

'''
Concrete implementations of RobotNavigator classes
'''

class CheatNav(RobotNavigator):
    '''
    Navigate by stealing simulated state vector from a RoboSim interface
    '''
    stateheading=RobotNavigator.stateheading+",tPPS,tNextPPPS,tPos,tNextPos"
    def __init__(self,interface,posInterval=1.0,ppsDelay=0.4):
        RobotNavigator.__init__(self, interface)
        self.posInterval=posInterval
        self.tPos=0.0              #time that the current position became valid (delayed from PPS by ppsDelay) (not available to guidance)
        self.tPPS=0.0              #time that the last PPS happened
        self.tNextPos=float('nan') #time that the position will be updated
        self.tNextPPS=float('nan') #time that the next PPS will happen
        self.cachePos=array([0.0,0.0]) #Cached copy of position that was recorded at tPPS and will become available at tPos (not available to guidance)
        self.pos=copy(self.cachePos)
        self.ppsDelay=ppsDelay
        self.hasPPS=True
        self.hasNewPos=True #Presume we have been sitting at the starting line and have a fix
    def navigate(self):
        self.hasNewHdg=True
        self.heading=self.interface.heading
        self.t=self.interface.t
        if isnan(self.tNextPPS) or self.tNextPPS<self.t:
            self.cachePos=copy(self.interface.pos)
            if isnan(self.tNextPPS):
                self.tNextPPS=self.t
            self.tPPS=self.tNextPPS
            self.tNextPPS+=self.posInterval
            self.hasPPS=True
        if isnan(self.tNextPos) or self.tNextPos<self.t:
            self.pos=copy(self.cachePos)
            if isnan(self.tNextPos):
                self.tNextPos=self.t
            self.tPos=self.tNextPos
            self.tNextPos=(self.tPPS+self.posInterval+self.ppsDelay)
            self.hasNewPos=True
    def state(self):
        return (RobotNavigator.state(self)
                +",%0.6f" % self.tPPS
                +",%0.6f" % self.tNextPPS
                +",%0.6f" % self.tPos
                +",%0.6f" % self.tNextPos)
                
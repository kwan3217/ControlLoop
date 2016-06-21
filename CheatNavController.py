from Robot import RobotNavigator
from math import isnan
from copy import copy

class CheatNav(RobotNavigator):
    stateheading=RobotNavigator.stateheading+",tNextPos"
    def __init__(self,interface,posInterval=1.0):
        RobotNavigator.__init__(self, interface)
        self.posInterval=posInterval
        self.tNextPos=float('nan')
    def navigate(self):
        self.hasNewHdg=True
        self.heading=self.interface.heading
        self.t=self.interface.t
        if isnan(self.tNextPos) or self.tNextPos<self.t:
            self.pos=copy(self.interface.pos)
            if isnan(self.tNextPos):
                self.tNextPos=0
            self.tNextPos+=self.posInterval
            self.hasNewPos=True
    def state(self):
        return RobotNavigator.state(self)+",%0.6f" % self.tNextPos
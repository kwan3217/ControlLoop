from ClosedLoopRobotController import ClosedLoopRobotController
from Robot import coerceHeadingRad
from math import atan2, isnan
from numpy.linalg import norm
from numpy import array, dot

class CheatNavController(ClosedLoopRobotController):
    def __init__(self,interface,posInterval=1.0):
        ClosedLoopRobotController.__init__(self, interface)
        self.pos       =array([float('nan'),float('nan')])
        self.heading   =       float('nan')
        self.posInterval=posInterval
        self.tLastPos=float('nan')
        self.hasNewPos=False
    def navigate(self):
        if isnan(self.tLastPos) or self.tLastPos+self.posInterval<self.interface.t:
            self.pos=self.interface.pos
            self.tLastPos=self.interface.t
            self.hasNewPos=True
        self.heading=self.interface.heading

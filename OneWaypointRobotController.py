from ClosedLoopRobotController import ClosedLoopRobotController
from math import atan2
from Robot import RobotController

class OneWaypointRobotController(ClosedLoopRobotController):
    def __init__(self,interface):
        ClosedLoopRobotController.__init__(self, interface)
        self.x=0.0
        self.y=0.0
        self.wptX=9724.6
        self.wptY=1728.0
    def navigate(self):
        self.x=self.interface.x
        self.y=self.interface.y
        self.heading=self.interface.heading
    def guide(self):
        self.cmdHeading=atan2((self.wptX-self.x),(self.wptY-self.y))

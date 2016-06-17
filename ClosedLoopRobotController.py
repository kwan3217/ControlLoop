from Robot import RobotController, coerceDHeadingRad

class ClosedLoopRobotController(RobotController):
    def __init__(self,interface):
        RobotController.__init__(self, interface)
        #Set up the part of the state vector estimate that the controller cares about
        self.heading=0.0;
        #Set up the guidance vector
        self.cmdHeading=0.0;
        #Set up the steering control constants
        self.P=-20;
    def control(self):
        stCmd=self.P*coerceDHeadingRad(self.heading-self.cmdHeading)
        if stCmd>1:
            stCmd=1
        elif stCmd<-1:
            stCmd=-1
        self.interface.steer(stCmd)
                
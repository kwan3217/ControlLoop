from Robot import RobotController, coerceDHeadingRad

class ClosedLoopRobotController(RobotController):
    def __init__(self,interface,P=-1):
        RobotController.__init__(self, interface)
        #Set up the part of the state vector estimate that the controller cares about
        self.heading=float('nan');
        #Set up the guidance vector
        self.cmdHeading=float('nan');
        #Set up the steering control constants
        self.P=P;
    def control(self):
        stCmd=self.P*coerceDHeadingRad(self.heading-self.cmdHeading)
        if stCmd>1:
            stCmd=1
        elif stCmd<-1:
            stCmd=-1
        self.interface.steer.write(stCmd)

from Robot import RobotController, coerceDHeadingRad

class ClosedLoopRobotController(RobotController):
    '''
    Robot controller which uses actual heading from estimated state vector 
    and commanded heading from guidance to calculate a steering command.
    
    Currently implements (only) Proportional Control
    '''
    stateheading=RobotController.stateheading
    def __init__(self,interface,nav,guide,P=-1):
        RobotController.__init__(self, interface, nav, guide)
        #Set up the steering control constants
        self.P=P;
    def control(self):
        '''
        Calculate the steering command
        '''
        if not self.guide.go:
            self.interface.throttle.write(0)
        if not self.nav.hasNewHdg:
            return
        stCmd=self.P*coerceDHeadingRad(self.nav.heading-self.guide.cmdHeading)
        self.interface.steer.write(stCmd)

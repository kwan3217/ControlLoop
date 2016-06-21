from Robot import RobotController, coerceDHeadingRad

'''
Concrete implementations of RobotController classes
'''

class OpenLoopRobotController(RobotController):
    def __init__(self,interface,nav=[],guide=[]):
        RobotController.__init__(self, interface,nav,guide)

    def control(self):
        """
        Calculate the steering and throttle command.
        
        This particular routine is open-loop and designed to pass the April 1
        test -- drive forward, turn right, then drive forward again
        """
        if self.interface.t<2:
            self.interface.throttle.write(0.25)
            self.interface.steer.write(0)
        elif self.interface.t<4:
            self.interface.steer.write(1)
        elif self.interface.t<6:
            self.interface.steer.write(0)
        else:
            self.interface.throttle.write(0)
           
class ClosedLoopRobotController(RobotController):
    '''
    Robot controller which uses actual heading from estimated state vector 
    and commanded heading from guidance to calculate a steering command.
    
    Currently implements (only) Proportional Guidance
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

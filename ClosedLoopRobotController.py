from Robot import RobotController, coerceDHeadingRad

class ClosedLoopRobotController(RobotController):
    '''
    Robot controller which uses actual heading from estimated state vector 
    and commanded heading from guidance to calculate a steering command.
    
    Currently implements (only) Proportional Control
    '''
    def __init__(self,interface,P=-1):
        '''
        Set up the controller
        Parameters:
          interface - A RobotInterface or descendant
          P - Proportional control constant. Should be negative, and controls 
              the "stiffness" of the controller.
        '''
        RobotController.__init__(self, interface)
        #Set up the part of the state vector estimate that the controller cares about
        self.heading=float('nan');
        #Set up the guidance vector
        self.cmdHeading=float('nan');
        #Set up the steering control constants
        self.P=P;
    def control(self):
        '''
        Calculate the steering command
        '''
        stCmd=self.P*coerceDHeadingRad(self.heading-self.cmdHeading)
        self.interface.steer.write(stCmd)

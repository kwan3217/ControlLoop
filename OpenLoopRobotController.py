from Robot import RobotController

class OpenLoopRobotController(RobotController):
    def __init__(self,interface):
        RobotController.__init__(self, interface)

    def control(self):
        """
        Calculate the steering and throttle command.
        
        This particular routine is open-loop and designed to pass the April 1
        test -- drive forward, turn right, then drive forward again
        """
        if self.interface.t<2:
            self.interface.throttle(0.25)
            self.interface.steer(0)
        elif self.interface.t<4:
            self.interface.steer(1)
        elif self.interface.t<6:
            self.interface.steer(0)
        else:
            self.interface.throttle(0)
           

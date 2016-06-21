from math import pi
from abc import ABC, abstractmethod
from numpy import array

def linterp(x0,y0,x1,y1,x):
    t=(x-x0)/(x1-x0)
    return y0*(1-t)+y1*t

def coerceHeadingRad(heading):
    '''
    Return a heading in radians, equivalent to the given heading, within the 
    proper range for a heading (between 0 to 2*pi).
    '''
    while heading<0:
        heading+=2*pi
    while heading>2*pi:
        heading-=2*pi
    return heading

def coerceDHeadingRad(dheading):
    '''
    Return a change in heading in radians, equivalent to the given change in heading, within the 
    proper range for a change in heading (between -pi to pi).
    '''
    while dheading<-pi:
        dheading+=2*pi
    while dheading>pi:
        dheading-=2*pi
    return dheading

class Servo(ABC):
    '''
    Represents a servo in either the simulator or actual hardware. 
    
    Since only a simulated servo can be read, we only have write in this
    class - read and step will be in the ServoSim class
    '''
    @abstractmethod
    def write(self, value):
        pass

class RobotInterface(ABC):
    '''
    Interface to the robot controls.
    
    On a real robot, this will create and transmit servo commands. A simulated
    robot will use the same interface, so that the same RoboController can be
    used to drive either a simulated or real robot.
    ''' 
    def __init__(self,steer,throttle):
        self.steer=steer
        self.throttle=throttle
        
class RobotNavigator(ABC):
    '''
    Maintain the state vector of the robot
    '''
    stateheading='nt,nposX,nposY,nHeading'
    def __init__(self,interface):
        self.interface=interface
        self.t        =0.0
        self.pos      =array([float('nan'),float('nan')])
        self.heading  =       float('nan')
        self.hasNewPos=False
        self.hasNewHdg=False
    @abstractmethod
    def navigate(self):        
        pass
    def state(self):
        '''
        Return a string describing the navigator state.
        '''
        return ("%0.6f," % self.t
               +"%0.6f,%0.6f," % tuple(self.pos)
               +"%0.6f," % self.heading)
    
class RobotGuidance(ABC):
    '''
    Maintain the state vector of the robot
    '''
    stateheading='cmdHeading'
    def __init__(self,nav):
        self.nav=nav
        self.cmdHeading=float('nan')
        self.go=True
    @abstractmethod
    def guide(self):        
        pass
    def state(self):
        '''
        Return a string describing the guidance state.
        '''
        return "%0.6f" % self.cmdHeading
class RobotController(ABC):
    '''
    Maintains the state vector estimate of the robot, calculates the giudance values,
    and executes the control loop for a robot.
    
    Member fields include the estimated state vector (maintained by navigate)
    and the guidance command vector (maintained by guide). Control can read 
    both of these, and is expected to generate steering commands and send them
    to the associated RobotInterface  
    '''
    stateheading=""
    def __init__(self,interface,nav,guide):
        '''
        set instance fields (treated as variables, independent if multiple instances)
        '''
        self.nav=nav                    #expected to be a RobotNavigator
        self.guide=guide                #expected to be a RobotGuidance
        self.interface=interface        #expected to be a RobotInterface

    @abstractmethod
    def control(self):
        pass
    
    def state(self):
        '''
        Return a string describing the controller state.
        '''
        return '' 

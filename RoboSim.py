from math import sin,cos,tan,pi
from abc import ABC, abstractmethod

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

class RoboController(ABC):
    @abstractmethod
    def navigate(self):
        pass
    
    @abstractmethod
    def guide(self):
        pass
    
    @abstractmethod
    def control(self):
        pass
    
class RoboSim(object):
    #set class fields (treated as constants)
    tickSize=1.0/1024  #time step size in seconds. Done this way to be an exactly representable number in floating-point
    wheelbase=30       #distance along vehicle axis between center of back axle and center of rotation of steering joint on front axle, cm'
    maxSt=0.2          #steering stop in radians from center
    stSlewSpeed=1      #steering slew speed in rad/s
    maxSp=500          #maximum speed in cm'/s - presumed to be the same forward and reverse
    spSlewSpeed=500    #speed change speed (acceleration) in cm'/s^2
    spDeadZone=80      #Minimum speed in cm'/s - commanded speeds with absolute value less than this get coerced to zero
    def __init__(self):
        '''
        set instance fields (treated as variables, independent if multiple instances)
        '''
        self.t=0        #Current time (useful for printing state, visible to NGC)
        self.x=0        #Easting in cm'
        self.y=0        #Northing in cm'
        self.heading=0  #heading in radians east of true North
        self.spCmd=0    #Commanded throttle in range [-1,1]
        self.sp=0       #Actual speed in cm'/s
        self.stCmd=0    #Steering in range [-1,1] where -1 is full left, +1 is full right
        self.st=0       #actual steering angle in radians right of center
        self.f = open('robosim.csv','w')
        print("t,x,y,hdg,spCmd,sp,stCmd,st",file=self.f)

    def stepSim(self):
        """
        Propagate the robot actual state by one time step.
        
        This routine does the following in order:
        * Adusts the speed towards the commanded speed if necessary
        * Adjusts the steering towards the commanded steering if necessary
        * Calculates the new heading based on the steering and speed
        * Calculates the velocity vector
        * Updates the position vector
        """
        # adjust speed
        cmdSp=self.spCmd*self.maxSp
        if abs(cmdSp)<self.spdDeadZone:
            #enforce throttle dead zone
            cmdSp=0
        if abs(cmdSp-self.sp)<self.spSlewSpeed*self.tickSize:
            # less than one step from there, just set it
            self.sp=cmdSp
        elif cmdSp>self.sp:
            #speed up
            self.sp+=self.spSlewSpeed*self.tickSize
        else:
            #slow down
            self.sp-=self.spSlewSpeed*self.tickSize
        
        # adjust steering
        cmdSt=self.stCmd*self.maxSt
        if abs(cmdSt-self.st)<self.stSlewSpeed*self.tickSize:
            # less than one step from there, just set it
            self.st=cmdSt
        elif cmdSt>self.st:
            #turn more right
            self.st+=self.stSlewSpeed*self.tickSize
        else:
            #turn less right (more left)
            self.st-=self.stSlewSpeed*self.tickSize

        #Figure turning curvature from steering angle
        # * Radius of curvature is the distance from the vehicle centerline to the center of the turning circle
        # * Curvature is reciprocal of radius of curvature - zero when going straight (no singularities there)
        #Both the front and back wheels are tangent to circles centered
        #on the center of curvature. Therefore, a line perpendicular to both
        #is at the center of curvature. Imagine a two-wheeled vehicle (or one
        #set of front-and-back wheels on a 4-wheel robot) with +X towards the
        #right and +Y towards the front (body coordinates). The center of the 
        #back wheel is at the origin. In this frame, the center of curvature
        #is directly to the side of the back wheel, and therefore on the 
        #x-axis (y=mx+b,m=0,b=0). Also, the center of curvature is on the line
        #perpendicular to the steering wheels. This is on the line y=mx+b, 
        #b=wheelbase, m=0 if straight ahead, negative if turning right, -inf
        #if turned 90deg to the right, positive if turning left, and +inf if
        #90deg to left, therefore m=-tan(st) since steer is positive to the 
        #right.
        #So, solve 0=-tan(steer)*x+b
        #-b=-tan(steer)*x
        # b=tan(steer)*x
        # b/tan(steer)=x=r,   turning radius from back wheels.
        # tan(steer)/b=kappa, turning curvature (units of 1/cm')
        kappa=tan(self.st)/self.wheelbase #(units are radians (1/1) divided by cm'=1/cm', correct for curvature)
        
        #Figure yaw rate
        #The yaw rate is just the speed divided by the turning radius (so speed cm'/s multiplied by curvature 1/cm'=1/s)
        yRate=self.sp*kappa
       
        #Update heading
        self.heading+=yRate*self.tickSize
        self.heading=coerceHeadingRad(self.heading)
        
        #calculate velocity
        dx=self.sp*sin(self.heading)*self.tickSize
        dy=self.sp*cos(self.heading)*self.tickSize
        
        #update position
        self.x+=dx
        self.y+=dy
        
        #update tme
        self.t+=self.tickSize
        
    def print(self):
        """
        Print the robot current actual state.
        """    
        print("%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f" % (self.t,self.x,self.y,self.heading,self.spCmd,self.sp,self.stCmd,self.st),file=self.f)
        
    def control(self):
        """
        Calculate the steering and throttle command.
        
        This particular routine is open-loop and designed to pass the April 1
        test -- drive forward, turn right, then drive forward again
        """
        if self.t<2:
            self.spCmd=0.25
            self.stCmd=0
        elif self.t<4:
            self.spCmd=0.25
            self.stCmd=1
        elif self.t<6:
            self.stCmd=0.25
            self.stCmd=0
        else:
            self.stCmd=0
            self.spCmd=0
           
#Instantiate the robot                
robosim=RoboSim()
#Main loop - let the robot NGC run, then step the simulation
while robosim.t<8:
    robosim.control()
    robosim.stepSim()
    robosim.print()    

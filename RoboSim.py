from math import sin,cos,tan
from Robot import RobotInterface, coerceHeadingRad
from numpy import array

class RoboSim(RobotInterface):
    '''
    Simulated robot. This keeps track of the simulated state vector
    '''
    #set class fields (treated as constants)
    tickSize=1.0/64  #time step size in seconds. Done this way to be an exactly representable number in floating-point
    wheelbase=30       #distance along vehicle axis between center of back axle and center of rotation of steering joint on front axle, cm'
    maxSt=0.2          #steering stop in radians from center
    stSlewSpeed=1      #steering slew speed in rad/s
    maxSp=500          #maximum speed in cm'/s - presumed to be the same forward and reverse
    spSlewSpeed=500    #speed change speed (acceleration) in cm'/s^2
    spDeadZone=80      #Minimum speed in cm'/s - commanded speeds with absolute value less than this get coerced to zero
    def __init__(self,oufn,extrastate):
        '''
        set instance fields (treated as variables, independent if multiple instances)
        '''
        self.t=0        #Current time (useful for printing state, visible to NGC)
        self.pos=array([0.0,0.0])  #Position relative to starting point in cm'
        self.heading=0  #heading in radians east of true North
        self.spCmd=0    #Commanded throttle in range [-1,1]
        self.sp=0       #Actual speed in cm'/s
        self.stCmd=0    #Steering in range [-1,1] where -1 is full left, +1 is full right
        self.st=0       #actual steering angle in radians right of center
        self.f = open(oufn,'w')
        print("t,x,y,hdg,spCmd,sp,stCmd,st,"+extrastate,file=self.f)
    def steer(self, steering):
        self.stCmd=steering
    def throttle(self, throttle):
        self.spCmd=throttle
    def stepSim(self):
        """
        Propagate the robot simulated state by one time step.
        
        This routine does the following in order:
        * Adusts the speed towards the commanded speed if necessary
        * Adjusts the steering towards the commanded steering if necessary
        * Calculates the new heading based on the steering and speed
        * Calculates the velocity vector
        * Updates the position vector
        """
        # adjust speed
        cmdSp=self.spCmd*self.maxSp
        if abs(cmdSp)<self.spDeadZone:
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

        #Figure turning curvature and yaw rate from steering angle - see 
        #https://github.com/kwan3217/RoboSim/wiki/Turning-Circle
        kappa=self.st/self.wheelbase #(units are radians (1/1) divided by cm'=1/cm', correct for curvature)
        dbeta=self.sp*kappa
       
        #Update heading
        self.heading+=dbeta*self.tickSize
        self.heading=coerceHeadingRad(self.heading)
        
        #calculate velocity
        v=self.sp*array([sin(self.heading),cos(self.heading)])*self.tickSize
        
        #update position
        self.pos+=v
        
        #update time
        self.t+=self.tickSize
        
    def print(self,extrastate):
        """
        Print the robot current actual state.
        """    
        print("%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%s" % (self.t,self.pos[0],self.pos[1],self.heading,self.spCmd,self.sp,self.stCmd,self.st,extrastate),file=self.f)
        
            

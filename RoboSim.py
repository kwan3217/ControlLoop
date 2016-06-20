from math import sin,cos,tan
from Robot import RobotInterface, coerceHeadingRad, Servo, linterp
from numpy import array

class ServoSim(Servo):
    def __init__(self,cmdMin,cmdMax,actMin,actMax,slewRate,act0):
        self.cmdMin=cmdMin
        self.cmdMax=cmdMax
        self.actMin=cmdMin
        self.actMax=actMax
        self.slewRate=slewRate
        self.act=act0
        self.write(linterp(actMin,cmdMin,actMax,cmdMax,act0))
    def write(self,cmd):
        if cmd>self.cmdMax:
            cmd=self.cmdMax
        elif cmd<self.cmdMin:
            cmd=self.cmdMin
        self.cmd=cmd
        self.cmdAct=linterp(self.cmdMin,self.actMin,self.cmdMax,self.actMax,cmd)
    def read(self):
        return self.act
    def step(self,dt):
        if abs(self.cmdAct-self.act)<self.slewRate*dt:
            # less than one step from there, just set it
            self.act=self.cmdAct
        elif self.cmdAct>self.act:
            #speed up
            self.act+=self.slewRate*dt
        else:
            #slow down
            self.act-=self.slewRate*dt

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
        self.steer=ServoSim(-1,1,-self.maxSt,self.maxSt,self.stSlewSpeed,0)
        self.throttle=ServoSim(-1,1,-self.maxSp,self.maxSp,self.spSlewSpeed,0)
        self.t=0        #Current time (useful for printing state, visible to NGC)
        self.pos=array([0.0,0.0])  #Position relative to starting point in cm'
        self.heading=0  #heading in radians east of true North
        self.f = open(oufn,'w')
        print("t,x,y,hdg,spCmd,sp,stCmd,st,"+extrastate,file=self.f)
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
        # Step the servos
        self.steer.step(self.tickSize)
        self.throttle.step(self.tickSize)
        
        #Figure turning curvature and yaw rate from steering angle - see 
        #https://github.com/kwan3217/RoboSim/wiki/Turning-Circle
        kappa=self.steer.read()/self.wheelbase #(units are radians (1/1) divided by cm'=1/cm', correct for curvature)
        dbeta=self.throttle.read()*kappa
        
        #Update heading
        self.heading+=dbeta*self.tickSize
        self.heading=coerceHeadingRad(self.heading)
        
        #calculate velocity
        v=self.throttle.read()*array([sin(self.heading),cos(self.heading)])*self.tickSize
        
        #update position
        self.pos+=v
        
        #update time
        self.t+=self.tickSize
        
    def print(self,extrastate):
        """
        Print the robot current actual state.
        """    
        print("%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%s" % (self.t,self.pos[0],self.pos[1],self.heading,self.throttle.cmd,self.throttle.act,self.steer.cmd,self.steer.act,extrastate),file=self.f)

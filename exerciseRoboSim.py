from RoboSim import RoboSim
from OpenLoopRobotController import OpenLoopRobotController

#Instantiate the robot simulator                
robosim=RoboSim('OpenLoop.csv')
robocontrol=OpenLoopRobotController(robosim)

#Main loop - let the robot NGC run, then step the simulation
while robosim.t<8:
    robocontrol.navigate()
    robocontrol.guide()
    robocontrol.control()
    robosim.stepSim()
    robosim.print()    

from RoboSim import RoboSim
from OneWaypointRobotController import OneWaypointRobotController

#Instantiate the robot simulator                
robosim=RoboSim('OneWaypoint.csv')
robocontrol=OneWaypointRobotController(robosim)
robosim.throttle(1)

#Main loop - let the robot NGC run, then step the simulation
while robosim.t<50:
    robocontrol.navigate()
    robocontrol.guide()
    robocontrol.control()
    robosim.stepSim()
    robosim.print()    

from RoboSim import RoboSim
from WaypointRobotController import WaypointRobotController
import numpy

#Instantiate the robot simulator                
robosim=RoboSim('OneWaypoint.csv',WaypointRobotController.stateheading)
robocontrol=WaypointRobotController(robosim,[numpy.array([4000.0,4000.0]),numpy.array([0.0,0.0])])
robosim.throttle(1)

#Main loop - let the robot NGC run, then step the simulation
while robosim.t<30:
    robocontrol.navigate()
    robocontrol.guide()
    robocontrol.control()
    robosim.stepSim()
    robosim.print(robocontrol.state())    

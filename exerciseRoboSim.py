from RoboSim import RoboSim
from Guidance import WaypointGuidance
from Control import ClosedLoopRobotController
from Navigation import CheatNav
import numpy

#Instantiate the robot simulator                
robosim=RoboSim('OneWaypoint.csv',CheatNav.stateheading+","+WaypointGuidance.stateheading+","+ClosedLoopRobotController.stateheading)
nav=CheatNav(robosim)
guide=WaypointGuidance(nav,[
    numpy.array([   0.0, 1500.0]),
    numpy.array([3500.0,-1500.0]),
    numpy.array([3500.0, 1500.0]),
    numpy.array([   0.0,-1500.0]),
    numpy.array([   0.0,    0.0])])
control=ClosedLoopRobotController(robosim,nav,guide)
robosim.throttle.write(1)

#Main loop - let the robot NGC run, then step the simulation
while robosim.throttle.read()>0 or robosim.throttle.cmd>0:
    nav.navigate()
    guide.guide()
    control.control()
    robosim.print(nav.state()+","+guide.state()+","+control.state())    
    robosim.stepSim()

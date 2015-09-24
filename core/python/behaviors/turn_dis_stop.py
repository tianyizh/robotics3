import memory, pose, commands, cfgstiff
import core
import mem_objects
from task import Task
from state_machine import *

class Playing(Task):
  def run(self):
    beacon_pink = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
    if beacon_pink.seen:
      #print "ball.visionBearing is %f!" % ball.visionBearing
      print "I see"
      commands.setStiffness()
      if beacon_pink.visionBearing > 0.20:
        commands.setWalkVelocity(0, 0, 0.3)
      elif beacon_pink.visionBearing < -0.20:
        commands.setWalkVelocity(0, 0, -0.3)
      else:
        #commands.setWalkVelocity(0, 0, 0)
        pose.Sit()
        commands.setStiffness(cfgstiff.Zero)
    else:
      commands.setStiffness()
      commands.setWalkVelocity(0, 0, -0.3)
      print "I don't see anything"
	

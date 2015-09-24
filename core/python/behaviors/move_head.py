import memory, commands
import core
import mem_objects
from task import Task
from state_machine import *
from time import sleep

class Playing(Task):
  def run(self):
    #commands.setWalkVelocity(0.2, 0, 0)
    #commands.setHeadPan(1, 1.0)
    ball = memory.world_objects.getObjPtr(core.WO_BALL)
    if ball.seen:
	  #memory.speech.say('Yes Yes Yes Yes')
      #commands.setHeadPan(1, 1.0)
      commands.setStiffness()
      print "ball.visionBearing is %f!" % ball.visionBearing
      commands.setHeadPan(ball.visionBearing, 0.1)
      print "Yes Yes Yes Yes"
    else:
	  #memory.speech.say('NO')
      print "NO"
	#commands.setStiffness()
	#commands.setHeadPan(-0.4, 3)
	#print "My new head roll value is %f!" % core.joint_values[core.HeadYaw]

	#commands.setWalkVelocity(0.2, 0, 0)
	    
	#commands.setHeadPan(0.5, 3)
	#print "My new head roll value is %f!" % core.joint_values[core.HeadYaw]

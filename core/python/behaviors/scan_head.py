import memory, pose, commands, cfgstiff, core
import mem_objects
from task import Task
from state_machine import *

direction = 1
class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        memory.speech.say("playing stand complete")
        self.finish()

  class ScanHead(Node):
    def run(self):
      global direction
      global angle
      global speed
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      angle = core.joint_values[core.HeadPan]
      speed = 8
      normalSpeed = 0.5
      commands.setStiffness()
      if ball.seen:
        commands.setHeadPan(ball.visionBearing, (ball.visionBearing-angle)/speed)
        self.finish()
      else:
        if direction == 1:
          commands.setHeadPan(1, (1-angle)/normalSpeed)
          print "positive direction" 
          print "angle %f!" % angle
          if angle > 0.9:
            print "change to negative direciton"
            direction = -1
        elif direction == -1:
          commands.setHeadPan(-1, (1+angle)/normalSpeed)
          print "positive direction" 
          print "angle %f!" % angle
          if angle < -0.9:
            print "change to positive direciton"
            direction = 1
        else:
            direction = 1;

  class Track(Node):
    def run(self):
      ball = memory.world_objects.getObjPtr(core.WO_BALL)
      print "I am tracking"
      if ball.seen:
        print "I see the ball"
        commands.setStiffness()
        #commands.setHeadPan(ball.visionBearing, 0.1)
        if ball.visionBearing > angle:
          commands.setHeadPan(ball.visionBearing, (ball.visionBearing-angle)/speed)
        else:
          commands.setHeadPan(ball.visionBearing, -(ball.visionBearing-angle)/speed)
          #tilt = -0.01*(ball.imageCenterY - 120)
          #commands.setHeadTilt(tilt)



  class HeadTurn(Node):
    def run(self):
      commands.setHeadPan(1, 3)
      if self.getTime() > 5.0:
        memory.speech.say("playing head turn complete")
        self.finish()

  class WalkStraight(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class WalkCurve(Node):
    def run(self):
      print "I am walking"
      commands.setWalkVelocity(0.5,0,0.3)

  class BodyTurn(Node):
    def run(self):
      commands.setWalkVelocity(0,0,0.3)
      if self.getTime() > 5.0:
        memory.speech.say("playing head turn complete")
        self.finish()

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    stand = self.Stand()
    scanhead = self.ScanHead()
    track = self.Track()
    walk = self.WalkCurve();
    #self.trans(stand, C, scanhead, C, track, C)
    self.trans(stand, C, scanhead, C, track, C, walk)

import memory, pose, commands, cfgstiff
from task import Task
from state_machine import *


class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        memory.speech.say("playing stand complete")
        self.finish()

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
    walkstraight = self.WalkStraight()
    walkcurve = self.WalkCurve()
    bodyturn = self.BodyTurn();
    headturn = self.HeadTurn();
    sit = pose.Sit()
    off = self.Off()
    self.trans(stand, T(50.0), off)

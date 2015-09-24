
import memory, commands
import core
import mem_objects
from task import Task
from state_machine import *

class Playing(Task):
  def run(self):
    commands.setStiffness()
    commands.setWalkVelocity(0, 0, -0.3)
    '''beacon_bp = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
    if beacon_bp.seen: 
      print "I see blue pink beacon" 
    
    beacon_pb = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE)
    if beacon_pb.seen: 
      print "I see blue pink beacon" 

    beacon_yb = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE)
    if beacon_yb.seen: 
      print "I see blue pink beacon"

    beacon_by = memory.world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW)
    if beacon_by.seen: 
      print "I see blue pink beacon"  

    beacon_py = memory.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
    if beacon_py.seen: 
      print "I see blue pink beacon"  

    beacon_yp = memory.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)
    if beacon_yp.seen: 
      print "I see blue pink beacon"  '''


   
	

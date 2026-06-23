from time import sleep, time
import math
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees import logging as log_tree
import robocasa.demos.singleton

'''global goal_state
global world_state
global associate_action_with_joint
next_action = ""
world_state = {}
goal_state = {}
associate_action_with_joint = {}'''

from enum import Enum
class Env_Locations(Enum):
  ROBOT_HAND = "ROBOT_HAND"
  FRIDGE = "FRIDGE"
  STOVE = "STOVE"
  SINK = "SINK"
  CABINET = "CABINET"
  COUNTER = "COUNTER"
  BOWL = "BOWL"
  POT = "POT"


class Env_Stove(Enum):
  FRONT_LEFT = "stovetop_right_group_1_knob_front_left_joint"
  FRONT_RIGHT = "stovetop_right_group_1_knob_front_right_joint"
  REAR_LEFT = "stovetop_right_group_1_knob_rear_left_joint"
  REAR_RIGHT = "stovetop_right_group_1_knob_rear_right_joint"
 

def determine_cabinet_closed(env):
  left_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")]
  right_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")]
  #global associate_action_with_joint
  robocasa.demos.singleton.associate_action_with_joint["is_cabinet_left_closed"] =  ("hingecabinet_3_main_group_1_leftdoorhinge",1, 1)
  robocasa.demos.singleton.associate_action_with_joint["is_cabinet_right_closed"] =  ("hingecabinet_3_main_group_1_rightdoorhinge",1, 1)

  


  is_left_closed = left_3 > -2.0e-07
  is_right_closed = right_3 < 2.0e-07

  if is_left_closed:
    robocasa.demos.singleton.world_state["is_cabinet_left_closed"] = True
  else:
    robocasa.demos.singleton.world_state["is_cabinet_left_closed"] = False
  
  if is_right_closed:
    robocasa.demos.singleton.world_state["is_cabinet_right_closed"] = True
  else:
    robocasa.demos.singleton.world_state["is_cabinet_right_closed"] = False

  return is_left_closed and is_right_closed


def determine_cabinet_opened(env):

  #global world_state
  #global associate_action_with_joint
  left_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")]
  right_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")]
  is_left_opened = left_3 < -1.0
  is_right_opened = right_3 > 1.0

  robocasa.demos.singleton.associate_action_with_joint["is_cabinet_left_opened"] =  ("hingecabinet_3_main_group_1_leftdoorhinge",1, 1)
  robocasa.demos.singleton.associate_action_with_joint["is_cabinet_right_opened"] =  ("hingecabinet_3_main_group_1_rightdoorhinge",1, 1)


  if is_left_opened:
    robocasa.demos.singleton.world_state["is_cabinet_left_opened"] = True
  else:
    robocasa.demos.singleton.world_state["is_cabinet_left_opened"] = False
  
  if is_right_opened:
    robocasa.demos.singleton.world_state["is_cabinet_right_opened"] = True
  else:
    robocasa.demos.singleton.world_state["is_cabinet_right_opened"] = False
  

  return is_left_opened and is_right_opened

def determine_cabinet_state(env, name, is_action=False):
  is_closed = determine_cabinet_closed(env)
  is_opened = determine_cabinet_opened(env)

  if is_action and robocasa.demos.singleton.world_state["is_robot_grip_occupied"]:
    return Status.FAILURE

  if name != "":
    sleep(1)
  if (is_closed and name == "is_cabinet_closed") or (is_opened and name == "is_cabinet_opened"):
    return Status.SUCCESS
  else:
    return Status.FAILURE

def determine_pot_cabinet(env, name=""):
  pot_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]]
  pot_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1]
  pot_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2]

  
  is_okay_pot_1 =  (pot_1 >= env.cabinet.pos[0] - env.cabinet.size[0]/2 + 0.12 and pot_1 <= env.cabinet.pos[0] + env.cabinet.size[0]/2 - 0.12)
  is_okay_pot_2 =  (pot_2 >= env.cabinet.pos[1] - env.cabinet.size[1]/2 + 0.12 and pot_2 <= env.cabinet.pos[1] + env.cabinet.size[1]/2 - 0.12)
  is_okay_pot_3 =  (pot_3 >= env.cabinet.pos[2] - env.cabinet.size[2]/2 + 0.06  and pot_3 <= env.cabinet.pos[2] + env.cabinet.size[2]/2 - 0.06 )
  #global world_state
  #global associate_action_with_joint
  robocasa.demos.singleton.associate_action_with_joint["pot_location"] = ("pot_joint0", 7, 6)
  
  if is_okay_pot_1 and is_okay_pot_2 and is_okay_pot_3:
    robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.CABINET
  elif robocasa.demos.singleton.world_state["pot_location"] == Env_Locations.CABINET:
    robocasa.demos.singleton.world_state["pot_location"] = None
  if name != "":
    sleep(1)
    if (is_okay_pot_1 and is_okay_pot_2 and is_okay_pot_3 and name=="is_pot_there_in_cabinet") or ((not is_okay_pot_1 or not is_okay_pot_2 or is_okay_pot_3) and name=="is_pot_not_there_in_cabinet"):
      return Status.SUCCESS
    else:
      return Status.FAILURE

def determine_robot_pot_grip(env, name="",  ignore_states=False, state=None):
  if name != "is_pot_in_robot_grip" and name != "is_pot_not_in_robot_grip" and name != "":
    return Status.FAILURE
  if state is not None:
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import reset_to
    reset_to(env, {"states": state})
  
  diff_x_axis_pot = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[0] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]])
  diff_y_axis_pot = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[1] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1])
  diff_z_axis_pot = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[2] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2])
  finger_joint1 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint1")])
  finger_joint2 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint2")])

  if name != "":
    sleep(1)

  if (finger_joint1 > 0.01 and finger_joint1 < 0.03) and (finger_joint2 > 0.01 and finger_joint2 < 0.03):
    if (diff_x_axis_pot<0.15 and diff_y_axis_pot < 0.15 and diff_z_axis_pot < 0.15):
      if ignore_states == False:
          robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.ROBOT_HAND
          robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = True
      if name == "is_pot_in_robot_grip":
        return Status.SUCCESS
      else:
        return Status.FAILURE
    else:
      if ignore_states == False:
          robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
      if ignore_states==False and robocasa.demos.singleton.world_state["pot_location"] == Env_Locations.ROBOT_HAND:
          robocasa.demos.singleton.world_state["pot_location"] = None
      if name == "is_pot_in_robot_grip":
        return Status.FAILURE
      else:
        return Status.SUCCESS

  else:
    if ignore_states == False:
        robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
    if ignore_states==False and robocasa.demos.singleton.world_state["pot_location"] == Env_Locations.ROBOT_HAND:
        robocasa.demos.singleton.world_state["pot_location"] = False
    if name == "is_pot_in_robot_grip":
     return Status.FAILURE
    else:
     return Status.SUCCESS

def determine_robot_egg1_grip(env, name="", ignore_states=False, state = None):
  if name != "is_egg1_in_robot_grip" and name != "is_egg1_not_in_robot_grip" and name != "":
    return Status.FAILURE
  if state is not None:
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import reset_to
    reset_to(env, {"states": state})
  diff_x_axis_egg1 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[0] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("egg1_joint0")[0]])
  diff_y_axis_egg1 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[1] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("egg1_joint0")[0]+1])
  diff_z_axis_egg1 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[2] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("egg1_joint0")[0]+2])
  finger_joint1 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint1")])
  finger_joint2 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint2")])

  robocasa.demos.singleton.associate_action_with_joint["egg1_location"] = ("egg1_joint0", 7, 6)


  if name != "":
    sleep(1)
  

  if (finger_joint1 > 0.01 and finger_joint1 < 0.03) and (finger_joint2 > 0.01 and finger_joint2 < 0.03):
    if (diff_x_axis_egg1 < 0.05 and diff_y_axis_egg1 < 0.05 and diff_z_axis_egg1 < 0.05):
      if ignore_states == False:
        robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = True
        robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.ROBOT_HAND
      if name == "is_egg1_in_robot_grip":
        return Status.SUCCESS
      else:
        return Status.FAILURE
    else:
      if ignore_states == False:
          robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
      if ignore_states==False and robocasa.demos.singleton.world_state["egg1_location"]==Env_Locations.ROBOT_HAND:
          robocasa.demos.singleton.world_state["egg1_location"] = None
          
   
      if name == "is_egg1_in_robot_grip":
        return Status.FAILURE
      else:
        return Status.SUCCESS

  else:
    if ignore_states == False:
        robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
    if ignore_states==False and robocasa.demos.singleton.world_state["egg1_location"]==Env_Locations.ROBOT_HAND:
        robocasa.demos.singleton.world_state["egg1_location"] = None

    if name == "is_egg1_in_robot_grip":
     return Status.FAILURE
    else:
     return Status.SUCCESS

def determine_robot_egg2_grip(env, name="", ignore_states=False, state = None):
  if name != "is_egg2_in_robot_grip" and name != "is_egg2_not_in_robot_grip" and name != "":
    return Status.FAILURE
  if state is not None:
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import reset_to
    reset_to(env, {"states": state})
  diff_x_axis_egg1 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[0] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("egg2_joint0")[0]])
  diff_y_axis_egg1 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[1] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("egg2_joint0")[0]+1])
  diff_z_axis_egg1 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[2] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("egg2_joint0")[0]+2])
  finger_joint1 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint1")])
  finger_joint2 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint2")])
  robocasa.demos.singleton.associate_action_with_joint["egg2_location"] = ("egg2_joint0", 7, 6)


  if name != "":
    sleep(1)
  

  if (finger_joint1 > 0.01 and finger_joint1 < 0.03) and (finger_joint2 > 0.01 and finger_joint2 < 0.03):
    if (diff_x_axis_egg1 < 0.05 and diff_y_axis_egg1 < 0.05 and diff_z_axis_egg1 < 0.05):
      if ignore_states == False:
        robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = True
        robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.ROBOT_HAND
      if name == "is_egg2_in_robot_grip":
        return Status.SUCCESS
      else:
        return Status.FAILURE
    else:
      if ignore_states == False:
          robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
      if ignore_states==False and robocasa.demos.singleton.world_state["egg2_location"]==Env_Locations.ROBOT_HAND:
          robocasa.demos.singleton.world_state["egg2_location"] = None
          
   
      if name == "is_egg2_in_robot_grip":
        return Status.FAILURE
      else:
        return Status.SUCCESS

  else:
    if ignore_states == False:
        robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
    if ignore_states==False and robocasa.demos.singleton.world_state["egg2_location"]==Env_Locations.ROBOT_HAND:
        robocasa.demos.singleton.world_state["egg2_location"] = None

    if name == "is_egg2_in_robot_grip":
     return Status.FAILURE
    else:
     return Status.SUCCESS


def determine_robot_egg_grip(env, name="", ignore_states=False, state = None, check_egg="egg1"):
  if name != f"is_{check_egg}_in_robot_grip" and name != f"is_{check_egg}_not_in_robot_grip" and name != "":
    return Status.FAILURE  

  if state is not None:
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import reset_to
    reset_to(env, {"states": state})
  
  diff_x_axis_egg2 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[0] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{check_egg}_joint0")[0]])
  diff_y_axis_egg2 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[1] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{check_egg}_joint0")[0]+1])
  diff_z_axis_egg2 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[2] - env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{check_egg}_joint0")[0]+2])
  finger_joint1 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint1")])
  finger_joint2 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint2")])
  if name != "":
    sleep(1)

  robocasa.demos.singleton.associate_action_with_joint[f"{check_egg}_location"] = (f"{check_egg}_joint0", 7, 6)
  if (finger_joint1 > 0.01 and finger_joint1 < 0.03) and (finger_joint2 > 0.01 and finger_joint2 < 0.03):
    if (diff_x_axis_egg2 <0.05 and diff_y_axis_egg2 < 0.05 and diff_z_axis_egg2 < 0.05):
      if ignore_states == False:
          robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = True
          robocasa.demos.singleton.world_state[f"{check_egg}_location"] = Env_Locations.ROBOT_HAND
      if name == f"is_{check_egg}_in_robot_grip":
        return Status.SUCCESS
      else:
        return Status.FAILURE
    else:
      if ignore_states == False:
        robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
      if ignore_states == False and robocasa.demos.singleton.world_state[f"{check_egg}_location"] == Env_Locations.ROBOT_HAND:
          robocasa.demos.singleton.world_state[f"{check_egg}_location"] == None
      if name == f"is_{check_egg}_in_robot_grip":
        return Status.FAILURE
      else:
        return Status.SUCCESS

  else:
    if ignore_states == False:
        robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False
    if ignore_states == False and robocasa.demos.singleton.world_state[f"{check_egg}_location"] == Env_Locations.ROBOT_HAND:
        robocasa.demos.singleton.world_state[f"{check_egg}_location"] == None

    if name == f"is_{check_egg}_in_robot_grip":
     return Status.FAILURE
    else:
     return Status.SUCCESS


def determine_sink_condition(env, name=""):
  sink_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("sink_island_group_1_handle_joint")] 
  if name != "":
    sleep(1) 
  
  robocasa.demos.singleton.associate_action_with_joint["is_sink_opened"] = ("sink_island_group_1_handle_joint", 1, 1)
  
  if (sink_1 < 0.4):
    robocasa.demos.singleton.world_state["is_sink_opened"] = False
  elif (sink_1 >= 0.4):
    robocasa.demos.singleton.world_state["is_sink_opened"] = True


  if (sink_1 < 0.4 and name == "is_sink_closed") or (sink_1 >= 0.4 and name == "is_sink_opened") :
    
    return Status.SUCCESS
  else:
    return Status.FAILURE

def determine_pot_sink(env, name="", state=None):

  if state is None:
    pot_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]]
    pot_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1]
    pot_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2]
  else:
    pot_1 = state[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1]
    pot_2 = state[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2]
    pot_3 = state[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+3]
  
  is_okay_pot_1 =  (pot_2 >= -4.4 and pot_2 <= -4.02)
  is_okay_pot_2 =  (pot_1 >= 2.6 and pot_1 <= 2.71)
  is_okay_pot_3 =  (pot_3 >= 0.71  and pot_3 <= 0.91)
  if name != "":
    sleep(1)
  
  if (is_okay_pot_1 and is_okay_pot_2 and is_okay_pot_3):
    robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.SINK
  elif (not is_okay_pot_1 or not is_okay_pot_2 or not is_okay_pot_3) and robocasa.demos.singleton.world_state["pot_location"] == Env_Locations.SINK:
    robocasa.demos.singleton.world_state["pot_location"] = None
  if (is_okay_pot_1 and is_okay_pot_2 and is_okay_pot_3 and (name == "is_pot_in_sink" or name == "")) or ((not is_okay_pot_1 or not is_okay_pot_2 or not is_okay_pot_3) and name == "is_pot_not_in_sink") :
    return Status.SUCCESS
  else:
    return Status.FAILURE
  
def determine_egg_pot(env, name="", should_sleep=True):
  egg_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-3]}_joint0")[0]]
  egg_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-3]}_joint0")[0]+1]
  egg_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-3]}_joint0")[0]+2]

  bowl_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]] - egg_1
  bowl_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1] - egg_2
  bowl_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2] - egg_3
  import math
  elnt_sqrt = math.sqrt(bowl_1*bowl_1+bowl_2*bowl_2)
  is_okay_egg_1 =  (elnt_sqrt < 0.0845 and bowl_3 > -0.128 and bowl_3 <0)

  if should_sleep:
    sleep(1)
    
  robocasa.demos.singleton.associate_action_with_joint[f"{name.split('_')[-3]}_location"] = (f"{name.split('_')[-3]}_joint0", 7, 6)
  if (not is_okay_egg_1 and "_not_" in name) or  (is_okay_egg_1 and "_not_" not in name):
    if is_okay_egg_1 and "_not_" not in name:
      if "egg1" in name:
        robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.POT
        robocasa.demos.singleton.world_state["is_pot_occupied_egg1"] = True
      elif "egg2" in name:
        robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.POT
        robocasa.demos.singleton.world_state["is_pot_occupied_egg2"] = True
    else:
      if "egg1" in name and robocasa.demos.singleton.world_state["egg1_location"]==Env_Locations.POT:
        robocasa.demos.singleton.world_state["egg1_location"] = None
      elif "egg2" in name and robocasa.demos.singleton.world_state["egg2_location"]==Env_Locations.POT:
        robocasa.demos.singleton.world_state["egg2_location"] = None
      
      if robocasa.demos.singleton.world_state["egg1_location"] != Env_Locations.POT and robocasa.demos.singleton.world_state["egg2_location"] != Env_Locations.POT:
        robocasa.demos.singleton.world_state["is_pot_occupied_egg1"] = False
        robocasa.demos.singleton.world_state["is_pot_occupied_egg2"] = False
      
    
    return Status.SUCCESS
  else:
    if is_okay_egg_1:
      if "egg1" in name:
          robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.POT
          robocasa.demos.singleton.world_state["is_pot_occupied_egg1"] = True
      elif "egg2" in name:
          robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.POT
          robocasa.demos.singleton.world_state["is_pot_occupied_egg2"] = True
    else:
      if "egg1" in name and robocasa.demos.singleton.world_state["egg1_location"]==Env_Locations.POT:
        robocasa.demos.singleton.world_state["egg1_location"] = None
      elif "egg2" in name and robocasa.demos.singleton.world_state["egg2_location"]==Env_Locations.POT:
        robocasa.demos.singleton.world_state["egg2_location"] = None

      if robocasa.demos.singleton.world_state["egg1_location"] != Env_Locations.POT and robocasa.demos.singleton.world_state["egg2_location"] != Env_Locations.POT:
        robocasa.demos.singleton.world_state["is_pot_occupied_egg1"] = False
        robocasa.demos.singleton.world_state["is_pot_occupied_egg2"] = False


    return Status.FAILURE
  
def determine_stove_state(env, name="", do_not_modify=False):
  is_turn_on_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("stovetop_right_group_1_knob_front_left_joint")] >= 0.35
  is_turn_on_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("stovetop_right_group_1_knob_front_right_joint")] >= 0.35
  is_turn_on_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("stovetop_right_group_1_knob_rear_left_joint")] >= 0.35
  is_turn_on_4 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("stovetop_right_group_1_knob_rear_right_joint")] >= 0.35

  if name != "":
    sleep(1)

  if name == "":
    robocasa.demos.singleton.associate_action_with_joint["stove.is_active"] = ("stovetop_right_group_1_knob_front_left_joint", 4, 4)
    if is_turn_on_1 and Env_Stove.FRONT_LEFT not in robocasa.demos.singleton.world_state["stove.is_active"]:
      robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.FRONT_LEFT)
    elif is_turn_on_2 and Env_Stove.FRONT_RIGHT not in robocasa.demos.singleton.world_state["stove.is_active"]:
      robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.FRONT_RIGHT)
    elif is_turn_on_3 and Env_Stove.REAR_LEFT not in robocasa.demos.singleton.world_state["stove.is_active"]:
      robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.REAR_LEFT)
    elif is_turn_on_4 and Env_Stove.REAR_RIGHT not in robocasa.demos.singleton.world_state["stove.is_active"]:
      robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.REAR_RIGHT)


  if (not is_turn_on_1  and name == "is_not_stove_turn_on" and (do_not_modify or Env_Stove.FRONT_LEFT in robocasa.demos.singleton.world_state["stove.is_active"])) or (is_turn_on_1 and name=="is_stove_turn_on" and (do_not_modify or Env_Stove.FRONT_LEFT not in robocasa.demos.singleton.world_state["stove.is_active"])):
    if not do_not_modify:
      if Env_Stove.FRONT_LEFT not in robocasa.demos.singleton.world_state["stove.is_active"]:
        robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.FRONT_LEFT)
      else:
        robocasa.demos.singleton.world_state["stove.is_active"].remove(Env_Stove.FRONT_LEFT)
    return Status.SUCCESS
  elif (not is_turn_on_2  and name == "is_not_stove_turn_on" and Env_Stove.FRONT_RIGHT not in robocasa.demos.singleton.world_state["stove.is_active"]) or (is_turn_on_2 and name=="is_stove_turn_on" and Env_Stove.FRONT_RIGHT in robocasa.demos.singleton.world_state["stove.is_active"]):
    if not do_not_modify:
      if Env_Stove.FRONT_RIGHT not in robocasa.demos.singleton.world_state["stove.is_active"]:
        robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.FRONT_RIGHT)
      else:
        robocasa.demos.singleton.world_state["stove.is_active"].remove(Env_Stove.FRONT_RIGHT)
    return Status.SUCCESS
  elif (not is_turn_on_3  and name == "is_not_stove_turn_on" and Env_Stove.REAR_LEFT not in robocasa.demos.singleton.world_state["stove.is_active"]) or (is_turn_on_3 and name=="is_stove_turn_on" and Env_Stove.REAR_LEFT in robocasa.demos.singleton.world_state["stove.is_active"]):
    if not do_not_modify:
      if Env_Stove.REAR_LEFT not in robocasa.demos.singleton.world_state["stove.is_active"]:
        robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.REAR_LEFT)
      else:
        robocasa.demos.singleton.world_state["stove.is_active"].remove(Env_Stove.REAR_LEFT)
    return Status.SUCCESS
  elif (not is_turn_on_4  and name == "is_not_stove_turn_on" and Env_Stove.REAR_RIGHT not in robocasa.demos.singleton.world_state["stove.is_active"]) or (is_turn_on_4 and name=="is_stove_turn_on" and Env_Stove.REAR_RIGHT in robocasa.demos.singleton.world_state["stove.is_active"]):
    if not do_not_modify:
      if Env_Stove.REAR_RIGHT not in robocasa.demos.singleton.world_state["stove.is_active"]:
        robocasa.demos.singleton.world_state["stove.is_active"].append(Env_Stove.REAR_RIGHT)
      else:
        robocasa.demos.singleton.world_state["stove.is_active"].remove(Env_Stove.REAR_RIGHT)
    return Status.SUCCESS
  else:
    return Status.FAILURE
  

def determine_is_pot_in_stove(env, name="", act_states=[], is_list=True):  
  pot2_x = None
  pot2_y = None
  pot2_z = None

  if is_list:
    if len(act_states) == 0:
      pot2_x = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]]
      pot2_y = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1]
      pot2_z = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2]    
    else:
      pot2_x = act_states[-1][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1]
      pot2_y = act_states[-1][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2]
      pot2_z = act_states[-1][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+3]
  else:
    pot2_x = act_states[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1]
    pot2_y = act_states[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2]
    pot2_z = act_states[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+3]


  if is_list:
    sleep(1)
  robocasa.demos.singleton.associate_action_with_joint["stove.is_occupied"] = ("pot_joint0", 7, 6)

  if pot2_z > 0.92 or pot2_z < 0.924:
    if pot2_x > 5.3 and pot2_x < 5.47 and pot2_y>-4.75 and pot2_y < -4.58:
      robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.STOVE

      if Env_Stove.FRONT_LEFT not in robocasa.demos.singleton.world_state["stove.is_occupied"]:
        robocasa.demos.singleton.world_state["stove.is_occupied"].append(Env_Stove.FRONT_LEFT)

      if name == "is_pot_in_stove":
        return "stovetop_right_group_1_knob_front_left_joint", Status.SUCCESS
      elif name == "is_not_pot_in_stove":
        return "stovetop_right_group_1_knob_front_left_joint", Status.FAILURE
    elif pot2_x > 5.3 and pot2_x < 5.47 and pot2_y>-4.99 and pot2_y < -4.78:

      robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.STOVE
      if Env_Stove.FRONT_RIGHT not in robocasa.demos.singleton.world_state["stove.is_occupied"]:
        robocasa.demos.singleton.world_state["stove.is_occupied"].append(Env_Stove.FRONT_RIGHT)


      if name == "is_pot_in_stove":
        return "stovetop_right_group_1_knob_front_right_joint", Status.SUCCESS
      elif name == "is_not_pot_in_stove":
        return "stovetop_right_group_1_knob_front_right_joint", Status.FAILURE
    elif pot2_x >= 5.47 and pot2_x < 5.58 and pot2_y>-4.75 and pot2_y < -4.58:
      robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.STOVE
      if Env_Stove.REAR_LEFT not in robocasa.demos.singleton.world_state["stove.is_occupied"]:
        robocasa.demos.singleton.world_state["stove.is_occupied"].append(Env_Stove.REAR_LEFT)

      if name == "is_pot_in_stove":
        return "stovetop_right_group_1_knob_rear_left_joint", Status.SUCCESS
      elif name == "is_not_pot_in_stove":
        return "stovetop_right_group_1_knob_rear_left_joint", Status.FAILURE
    elif pot2_x >= 5.47 and pot2_x < 5.58 and pot2_y>-4.99 and pot2_y < -4.78:
      robocasa.demos.singleton.associate_action_with_joint["stove.is_active"] = ("pot_location", 7, 6)

      robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.STOVE
      if Env_Stove.REAR_RIGHT not in robocasa.demos.singleton.world_state["stove.is_occupied"]:
        robocasa.demos.singleton.world_state["stove.is_occupied"].append(Env_Stove.REAR_RIGHT)

      if name == "is_pot_in_stove":
        return "stovetop_right_group_1_knob_rear_right_joint", Status.SUCCESS
      elif name == "is_not_pot_in_stove":
        return "stovetop_right_group_1_knob_rear_right_joint", Status.FAILURE
    elif robocasa.demos.singleton.world_state["pot_location"] == Env_Locations.STOVE:
      robocasa.demos.singleton.world_state["pot_location"] = None
    
    if name=="":
      return None
    
    robocasa.demos.singleton.world_state["stove.is_occupied"] = []
    if name == "is_pot_in_stove":
      
      return None, Status.FAILURE
    elif name == "is_not_pot_in_stove":
      return None, Status.SUCCESS 
  else:
    robocasa.demos.singleton.world_state["stove.is_occupied"] = []
    if name == "is_pot_in_stove":
      return None, Status.FAILURE
    elif name == "is_not_pot_in_stove":
      return None, Status.SUCCESS 
    


def determine_pot_position_sink(env, name="", state=None):
  if state is None:
    sink_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("sink_island_group_1_spout_joint")]
  else:
    sink_1 = state[env.sim.model.get_joint_qpos_addr("sink_island_group_1_spout_joint")+1]

  if name != "":
    sleep(1)

  robocasa.demos.singleton.associate_action_with_joint["is_sink_positioned"] = ("sink_island_group_1_spout_joint", 1, 1)
  if (sink_1 >= 0.49):
    robocasa.demos.singleton.world_state["is_sink_positioned"] = True
  elif (sink_1 <0.49):
    robocasa.demos.singleton.world_state["is_sink_positioned"] = False

  if (sink_1 >= 0.49 and (name == "is_sink_in_pot_position" or name == "")) or (sink_1 <0.49 and name == "is_sink_not_in_pot_position"):
    return Status.SUCCESS
  else:
    return Status.FAILURE

def determine_fridge_state(env, name=""):
  fridge_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("fridgebottomfreezer_main_group_1_fridge_door_joint")]
  robocasa.demos.singleton.associate_action_with_joint["is_fridge_opened"] = ("fridgebottomfreezer_main_group_1_fridge_door_joint", 1, 1)
  robocasa.demos.singleton.associate_action_with_joint["is_fridge_closed"] = ("fridgebottomfreezer_main_group_1_fridge_door_joint", 1, 1)

  if name != "":
    sleep(1)

  if (fridge_1 <= 0.1):
    robocasa.demos.singleton.world_state["is_fridge_opened"] = False
    robocasa.demos.singleton.world_state["is_fridge_closed"] = True
  elif (fridge_1 >= 1.4):
    robocasa.demos.singleton.world_state["is_fridge_opened"] = True
    robocasa.demos.singleton.world_state["is_fridge_closed"] = False
  else:
    robocasa.demos.singleton.world_state["is_fridge_opened"] = False
    robocasa.demos.singleton.world_state["is_fridge_closed"] = False

    
  if (fridge_1 <= 0.1 and name == "is_fridge_closed") or (fridge_1 >= 1.4 and name == "is_fridge_opened"):
    return Status.SUCCESS
  else:
    return Status.FAILURE
  
def determine_egg_fridge(env, name):
  egg_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-2]}_joint0")[0]]
  egg_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-2]}_joint0")[0]+1]
  egg_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-2]}_joint0")[0]+2]

  
  is_okay_egg_1 =  (egg_1 >= env.fridge.pos[0] - env.fridge.size[0]/2 + 0.01 and egg_1 <= env.fridge.pos[0] + env.fridge.size[0]/2 - 0.01)
  is_okay_egg_2 =  (egg_2 >= env.fridge.pos[1] - env.fridge.size[1]/2 + 0.01 and egg_2 <= env.fridge.pos[1] + env.fridge.size[1]/2 - 0.01)
  is_okay_egg_3 =  (egg_3 >= env.fridge.pos[2] - env.fridge.size[2]/2  and egg_3 <= env.fridge.pos[2] + env.fridge.size[2]/2)

  sleep(1)
  robocasa.demos.singleton.associate_action_with_joint[f"{name.split('_')[-2]}_location"] = (f"{name.split('_')[-2]}_joint0", 7, 6)

  if (is_okay_egg_1 and is_okay_egg_2 and is_okay_egg_3):
    if "egg1" in name:
      robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.FRIDGE
    elif "egg2" in name:
      robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.FRIDGE
  else:
    if "egg1" in name and robocasa.demos.singleton.world_state["egg1_location"] == Env_Locations.FRIDGE :
      robocasa.demos.singleton.world_state["egg1_location"] = None
    elif "egg2" in name and robocasa.demos.singleton.world_state["egg2_location"] == Env_Locations.FRIDGE:
      robocasa.demos.singleton.world_state["egg2_location"] = None

  
  if (is_okay_egg_1 and is_okay_egg_2 and is_okay_egg_3 and "_not_" not in name) or ((not is_okay_egg_1 or not is_okay_egg_2 or not is_okay_egg_3 ) and "_not_" in name):
    return Status.SUCCESS
  else:
    return Status.FAILURE


def number_of_eggs(env, type="fridge", state=None):
  if type !="fridge" and type != "pot" and type !="bowl" and type!="robot_hand":
    return "", None
  if env == None:
    return "egg1", None
  eggs_num = list(filter(lambda x: x.startswith("egg") and "joint0" in x, env.sim.model.joint_names))
  number_okay = 0
  first_element_in_list = None

  if state is not None:
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import reset_to
    reset_to(env, {"states": state})


  for egg_name in eggs_num:
    egg1_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{egg_name}")[0]]
    egg1_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{egg_name}")[0]+1]
    egg1_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{egg_name}")[0]+2]
    is_okay_egg_1 = False
    if type == "fridge":
      is_okay_egg1_1 =  (egg1_1 >= env.fridge.pos[0] - env.fridge.size[0]/2 + 0.01 and egg1_1 <= env.fridge.pos[0] + env.fridge.size[0]/2 - 0.01)
      is_okay_egg1_2 =  (egg1_2 >= env.fridge.pos[1] - env.fridge.size[1]/2 + 0.01 and egg1_2 <= env.fridge.pos[1] + env.fridge.size[1]/2 - 0.01)
      is_okay_egg1_3 =  (egg1_3 >= env.fridge.pos[2] - env.fridge.size[2]/2  and egg1_3 <= env.fridge.pos[2] + env.fridge.size[2]/2)
      is_okay_egg_1 = is_okay_egg1_1 and is_okay_egg1_2 and is_okay_egg1_3
    elif type == "bowl":
      bowl_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("bowl1_joint0")[0]] - egg1_1
      bowl_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("bowl1_joint0")[0]+1] - egg1_2
      bowl_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("bowl1_joint0")[0]+2] - egg1_3
      import math
      elnt_sqrt = math.sqrt(bowl_1*bowl_1+bowl_2*bowl_2)
      is_okay_egg_1 =  (elnt_sqrt < 0.111 and bowl_3 > -0.512 and bowl_3 <0)
    elif type == "pot":
      bowl_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]] - egg1_1
      bowl_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1] - egg1_2
      bowl_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2] - egg1_3
      import math
      elnt_sqrt = math.sqrt(bowl_1*bowl_1+bowl_2*bowl_2)
      is_okay_egg_1 =  (elnt_sqrt < 0.0845 and bowl_3 > -0.128 and bowl_3 <0)
    elif type == "robot_hand":
      diff_x_axis_egg2 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[0] - egg1_1)
      diff_y_axis_egg2 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[1] - egg1_2)
      diff_z_axis_egg2 = abs(env.sim.data.get_site_xpos("gripper0_right_grip_site")[2] - egg1_3)
      finger_joint1 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint1")])
      finger_joint2 = abs(env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("gripper0_right_finger_joint2")])
      is_okay_egg_1 =  (finger_joint1 > 0.01 and finger_joint1 < 0.03) and (finger_joint2 > 0.01 and finger_joint2 < 0.03) 
      is_okay_egg_1 = is_okay_egg_1 and (diff_x_axis_egg2 <0.05 and diff_y_axis_egg2 < 0.05 and diff_z_axis_egg2 < 0.05)

    if is_okay_egg_1:
      number_okay+=1
      if first_element_in_list == None:
        first_element_in_list = egg_name

  if first_element_in_list == None:
    return None, number_okay
  return first_element_in_list.split("_")[0], number_okay



def determine_nr_eggs_fridge(env, name):
  
  _, number_okay = number_of_eggs(env, type="fridge")  
  sleep(1)
  if (number_okay >= 1 and name == "check_eggs_1"):
    return Status.SUCCESS
  else:
    return Status.FAILURE

def determine_egg_bowl(env, name):
  egg_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-3]}_joint0")[0]]
  egg_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-3]}_joint0")[0]+1]
  egg_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr(f"{name.split('_')[-3]}_joint0")[0]+2]
  bowl_1 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("bowl1_joint0")[0]] - egg_1
  bowl_2 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("bowl1_joint0")[0]+1] - egg_2
  bowl_3 = env.sim.data.qpos[env.sim.model.get_joint_qpos_addr("bowl1_joint0")[0]+2] - egg_3
  import math
  elnt_sqrt = math.sqrt(bowl_1*bowl_1+bowl_2*bowl_2)
  is_okay_egg_1 =  (elnt_sqrt < 0.111 and bowl_3 > -0.512 and bowl_3 <0)
  
  sleep(1)
  if (is_okay_egg_1):
    if "egg1" in name:
      robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.BOWL
    elif "egg2" in name:
      robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.BOWL
  else:
    if "egg1" in name and robocasa.demos.singleton.world_state["egg1_location"] == Env_Locations.BOWL:
      robocasa.demos.singleton.world_state["egg1_location"] = None
    elif "egg2" in name and robocasa.demos.singleton.world_state["egg2_location"] == Env_Locations.BOWL:
      robocasa.demos.singleton.world_state["egg2_location"] = None
  if (not is_okay_egg_1 and "_not_" in name) or  (is_okay_egg_1 and "_not_" not in name):
    return Status.SUCCESS
  else:
    return Status.FAILURE


def was_water_boiled(states, env):

  boiled_water = 0
  sleep(1)
  for i in range(len(states)):
    item_name, is_sucess = determine_is_pot_in_stove(env, "is_pot_in_stove", states[i], False)
    if is_sucess == Status.SUCCESS and states[i][env.sim.model.get_joint_qpos_addr(f"{item_name}")+1] >= 0.35:
      boiled_water+=1

  
  if boiled_water > 1000:
    robocasa.demos.singleton.world_state["is_water_boiled"] = True
    return Status.SUCCESS
  else:
    return Status.FAILURE
  
def were_eggs_boiled(states, env):

  boiled_eggs = 0
  sleep(1)

  for i in range(len(states)):
    if robocasa.demos.singleton.world_state["egg2_location"]  != Env_Locations.POT or determine_egg_pot(env, "is_egg2_in_pot", False) == Status.SUCCESS:
      if robocasa.demos.singleton.world_state["egg1_location"] != Env_Locations.POT or determine_egg_pot(env, "is_egg1_in_pot", False) == Status.SUCCESS:
        if robocasa.demos.singleton.world_state["egg2_location"]  != Env_Locations.POT and robocasa.demos.singleton.world_state["egg1_location"] != Env_Locations.POT:

          return Status.FAILURE
        item_name, is_sucess = determine_is_pot_in_stove(env, "is_pot_in_stove", states[i], False)

        if is_sucess == Status.SUCCESS and states[i][env.sim.model.get_joint_qpos_addr(f"{item_name}")+1] >= 0.35:
          boiled_eggs+=1
        else:
          boiled_eggs = 0


  if boiled_eggs > 1000:
    if robocasa.demos.singleton.world_state["egg1_location"] == Env_Locations.POT:
      robocasa.demos.singleton.world_state["is_egg1_boiled"] = True
    if robocasa.demos.singleton.world_state["egg2_location"] == Env_Locations.POT:
      robocasa.demos.singleton.world_state["is_egg2_boiled"] = True
    return Status.SUCCESS
  else:
    return Status.FAILURE
  
  

def was_water_spilled(states, env):

  spilled_water = 0
  sleep(1)
  for i in range(len(states)):
    pot_x = states[i][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+1]
    pot_y = states[i][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+2]
    pot_z = states[i][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+3]

    if pot_y > -4.33 and pot_y < -3.99 and pot_x > 2.6 and pot_x < 2.84 and pot_z >  0.71:
      spill_angle_1 = abs(states[i][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+5])
      spill_angle_2 = abs(states[i][env.sim.model.get_joint_qpos_addr("pot_joint0")[0]+6])

      if spill_angle_1 > 0.63 or spill_angle_2 > 0.63:
        spilled_water+=1
  
  if spilled_water > 200:
    robocasa.demos.singleton.world_state["is_water_boiled"] = False
    robocasa.demos.singleton.world_state["is_water_in_pot"] = False
    return Status.SUCCESS
  else:
    return Status.FAILURE
      

def was_water_filled(states, env):

  filled_water = 0
  sleep(1)

  for i in range(len(states)):
    
    if determine_pot_sink(env, "", states[i]) == Status.SUCCESS:
      if determine_pot_position_sink(env, "", states[i]) == Status.SUCCESS:
        filled_water+=1
    
  
  if filled_water > 200:
    robocasa.demos.singleton.world_state["is_water_in_pot"] = True
    return Status.SUCCESS
  else:
    return Status.FAILURE



class PickPotFromCabinetAction(Behaviour):
  def __init__(self):
    super().__init__("pick_pot_from_cabinet_better")
    self.conditions =  [
      Condition("is_robot_grip_not_occupied", "pick_pot_from_cabinet_better"),
      Condition("is_pot_there_in_cabinet", "pick_pot_from_cabinet_better")
    ]
  def setup(self):
    self.logger.debug(f"PickPotFromCabinetAction::setup")

  def initialise(self):
    self.logger.debug(f"PickPotFromCabinetAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_pot_cabinet(env, "is_pot_not_there_in_cabinet") == Status.SUCCESS:
      return determine_robot_pot_grip(env, "is_pot_in_robot_grip")
    return Status.FAILURE

  
  def terminate(self, new_status):
    self.logger.debug(f"PickPotFromCabinetAction::terminate to {new_status}")

class PutPotInSinkAction(Behaviour):
  def __init__(self):
    super().__init__("put_pot_in_sink")
    self.conditions =  [
      Condition("is_pot_in_robot_grip", "put_pot_in_sink")
    ]
  def setup(self):
    self.logger.debug(f"PutPotInSinkAction::setup")

  def initialise(self):
    self.logger.debug(f"PutPotInSinkAction::initialise")

  def update(self):
    
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_pot_sink(env, "is_pot_in_sink") == Status.SUCCESS:
      return determine_robot_pot_grip(env, "is_pot_not_in_robot_grip")
    return Status.FAILURE
  
  def terminate(self, new_status):
    self.logger.debug(f"PutPotInSinkAction::terminate to {new_status}")

class TurnOnSinkAction(Behaviour):
  def __init__(self):
    super().__init__("turn_on_sink")
    self.conditions =  [
      Condition("is_robot_grip_not_occupied", "turn_on_sink"),
      Condition("is_sink_closed", "turn_on_sink")
    ]
  def setup(self):
    self.logger.debug(f"TurnOnSinkAction::setup")

  def initialise(self):
    self.logger.debug(f"TurnOnSinkAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_sink_condition(env, "is_sink_opened")
  
  def terminate(self, new_status):
    self.logger.debug(f"TurnOnSinkAction::terminate to {new_status}")

class PositionFaucetInPotAction(Behaviour):
  def __init__(self):
    super().__init__("position_faucet_in_pot")
    self.conditions =  [
      Condition("is_robot_grip_not_occupied", "position_faucet_in_pot"),
      Condition("is_sink_not_in_pot_position", "position_faucet_in_pot")
    ]
  def setup(self):
    self.logger.debug(f"PositionFaucetInPotAction::setup")

  def initialise(self):
    self.logger.debug(f"PositionFaucetInPotAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_pot_position_sink(env, "is_sink_in_pot_position")
  
  def terminate(self, new_status):
    self.logger.debug(f"PositionFaucetInPotAction::terminate to {new_status}")

class PutWaterInPotAction(Behaviour):
  def __init__(self):
    super().__init__("put_water_in_pot")
    self.conditions =  [
      Condition("is_pot_in_sink", "put_water_in_pot"),
      Condition("is_sink_in_pot_position", "put_water_in_pot")
    ]
  def setup(self):
    self.logger.debug(f"PutWaterInPotAction::setup")

  def initialise(self):
    self.logger.debug(f"PutWaterInPotAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return was_water_filled(act_states, env)
  
  def terminate(self, new_status):
    self.logger.debug(f"PutWaterInPotAction::terminate to {new_status}")


class RepositionFaucetAction(Behaviour):
  def __init__(self):
    super().__init__("reposition_faucet")
    self.conditions = [
        Condition("is_robot_grip_not_occupied", "reposition_faucet"),
        Condition("is_sink_in_pot_position", "reposition_faucet")
    ]
  def setup(self):
    self.logger.debug(f"RepositionFaucetAction::setup")

  def initialise(self):
    self.logger.debug(f"RepositionFaucetAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_pot_position_sink(env, "is_sink_not_in_pot_position")
  
  def terminate(self, new_status):
    self.logger.debug(f"RepositionFaucetAction::terminate to {new_status}")

class TurnOffSinkAction(Behaviour):
  def __init__(self):
    super().__init__("turn_off_sink_better")
    self.conditions = [
        Condition("is_robot_grip_not_occupied", "turn_off_sink_better"),
        Condition("is_sink_opened", "turn_off_sink_better")
    ]
  def setup(self):
    self.logger.debug(f"TurnOffSinkAction::setup")

  def initialise(self):
    self.logger.debug(f"TurnOffSinkAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_sink_condition(env, "is_sink_closed")
  
  def terminate(self, new_status):
    self.logger.debug(f"TurnOffSinkAction::terminate to {new_status}")

class PickPotFromSinkAction(Behaviour):
  def __init__(self):
    super().__init__("pick_pot_from_sink_better")
    self.conditions = [
      Condition("is_robot_grip_not_occupied", "pick_pot_from_sink_better"),
      Condition("is_pot_in_sink", "pick_pot_from_sink_better")
    ]
  def setup(self):
    self.logger.debug(f"PickPotFromSinkAction::setup")

  def initialise(self):
    self.logger.debug(f"PickPotFromSinkAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_pot_sink(env, "is_pot_not_in_sink") == Status.SUCCESS:
      return determine_robot_pot_grip(env, "is_pot_in_robot_grip")
    return Status.FAILURE
  
  def terminate(self, new_status):
    self.logger.debug(f"PickPotFromSinkAction::terminate to {new_status}")


class OpenFridgeAction(Behaviour):
  def __init__(self):
    super().__init__("open_fridge")
    self.conditions = [
      Condition("is_robot_grip_not_occupied", "open_fridge"),
      Condition("is_fridge_closed", "open_fridge")
    ]
  def setup(self):
    self.logger.debug(f"OpenFridgeAction::setup")

  def initialise(self):
    self.logger.debug(f"OpenFridgeAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True

    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_fridge_state(env, "is_fridge_opened")
  
  def terminate(self, new_status):
    self.logger.debug(f"OpenFridgeAction::terminate to {new_status}")



class PickEggFromFridgeAction(Behaviour):
  def __init__(self):
    super().__init__("pick_egg_from_fridge")

    self.conditions = [
      Condition("is_robot_grip_not_occupied", self.name, modify_action=True),
      Condition("check_eggs_1", self.name),
    ]
  def setup(self):
    self.logger.debug(f"PickEggFromFridgeAction::setup")

  def initialise(self):
    self.logger.debug(f"PickEggFromFridgeAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    self.name = self.conditions[0].action
    self.egg_name = robocasa.demos.singleton.general_information["egg_name"]
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_robot_egg_grip(env, f"is_{self.egg_name}_in_robot_grip", check_egg=self.egg_name) == Status.SUCCESS:
      return determine_egg_fridge(env, f"is_not_fridge_{self.egg_name}_there")    
    return Status.FAILURE

  
  def terminate(self, new_status):
    self.logger.debug(f"PickEggFromFridgeAction::terminate to {new_status}")



class CloseFridgeAction(Behaviour):
  def __init__(self):
    super().__init__("close_fridge")
    self.conditions = [
      Condition("is_fridge_opened", "close_fridge")
    ]
  def setup(self):
    self.logger.debug(f"CloseFridgeAction::setup")

  def initialise(self):
    self.logger.debug(f"CloseFridgeAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_fridge_state(env, "is_fridge_closed")
  
  def terminate(self, new_status):
    self.logger.debug(f"CloseFridgeAction::terminate to {new_status}")


class PutEggInBowlAction(Behaviour):
  def __init__(self):
    super().__init__("put_egg_in_bowl")
    self.conditions = [
      Condition("is_egg_in_robot_grip", self.name, modify_action=True),
      Condition("is_not_egg_in_bowl", self.name)
    ]
  def setup(self):
    self.logger.debug(f"PutEggInBowlAction::setup")

  def initialise(self):
    self.logger.debug(f"PutEggInBowlAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    self.name = self.conditions[0].action
    self.egg_name = robocasa.demos.singleton.general_information["egg_name"]
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_egg_bowl(env, f"is_{self.egg_name}_in_bowl") == Status.SUCCESS:
      return determine_robot_egg_grip(env, f"is_{self.egg_name}_not_in_robot_grip", check_egg=self.egg_name)
    return Status.FAILURE

  
  def terminate(self, new_status):
    self.logger.debug(f"PutEggInBowlAction::terminate to {new_status}")



class PutPotInStoveAction(Behaviour):
  def __init__(self):
    super().__init__("put_pot_in_stove")
    self.conditions = [
        Condition("is_pot_in_robot_grip", "put_pot_in_stove")
    ]
  def setup(self):
    self.logger.debug(f"PutPotInStoveAction::setup")

  def initialise(self):
    self.logger.debug(f"PutPotInStoveAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_robot_pot_grip(env, "is_pot_not_in_robot_grip") == Status.SUCCESS:
      return determine_is_pot_in_stove(env, "is_pot_in_stove", act_states)[1]
    return Status.FAILURE
  
  def terminate(self, new_status):
    self.logger.debug(f"PutPotInStoveAction::terminate to {new_status}")


class TurnOnStoveAction(Behaviour):
  def __init__(self):
    super().__init__("turn_on_stove")

    self.conditions = [
        Condition("is_robot_grip_not_occupied", "turn_on_stove"),
        Condition("is_not_stove_turn_on", "turn_on_stove")
    ]
  def setup(self):
    self.logger.debug(f"TurnOnStoveAction::setup")

  def initialise(self):
    self.logger.debug(f"TurnOnStoveAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_stove_state(env, "is_stove_turn_on")
  
  def terminate(self, new_status):
    self.logger.debug(f"TurnOnStoveAction::terminate to {new_status}")


class BoilWaterAction(Behaviour):
  def __init__(self):
    super().__init__("boil_water")
    self.conditions = [
        Condition("is_stove_turn_on", "boil_water"),
        Condition("is_pot_in_stove", "boil_water"),
        Condition("is_egg1_not_in_pot", "boil_eggs"),
        Condition("is_egg2_not_in_pot", "boil_eggs")
    ]
  def setup(self):
    self.logger.debug(f"BoilWaterAction::setup")

  def initialise(self):
    self.logger.debug(f"BoilWaterAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return was_water_boiled(act_states, env)
  
  def terminate(self, new_status):
    self.logger.debug(f"BoilWaterAction::terminate to {new_status}")

class PickEggFromBowlAction(Behaviour):
  def __init__(self):
    super().__init__("pick_egg_from_bowl")
    self.conditions =  [
      Condition("is_robot_grip_not_occupied", self.name, modify_action=True),
      Condition("is_egg_in_bowl", self.name)
    ]
  def setup(self):
    self.logger.debug(f"PickEggFromBowlAction::setup")

  def initialise(self):
    self.logger.debug(f"PickEggFromBowlAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    self.name = self.conditions[0].action
    self.egg_name = robocasa.demos.singleton.general_information["egg_name"]
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_egg_bowl(env, f"is_not_{self.egg_name}_in_bowl") == Status.SUCCESS:
      return determine_robot_egg_grip(env, f"is_{self.egg_name}_in_robot_grip", check_egg=self.egg_name)
  
    return Status.FAILURE
  
  def terminate(self, new_status):
    self.logger.debug(f"PickEggFromBowlAction::terminate to {new_status}")
 

class PutEggInPotAction(Behaviour):
  def __init__(self):
    super().__init__("put_egg_in_pot")
    self.conditions =  [
      Condition("is_egg_in_robot_grip", "put_egg_in_pot", modify_action=True)
    ]
  def setup(self):
    self.logger.debug(f"PutEggInPotAction::setup")

  def initialise(self):
    self.logger.debug(f"PutEggInPotAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    self.name = self.conditions[0].action
    self.egg_name = robocasa.demos.singleton.general_information["egg_name"]
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if robocasa.demos.singleton.world_state["is_water_boiled"] == False:
      return Status.FAILURE
    if determine_egg_pot(env, f"is_{self.egg_name}_in_pot") == Status.SUCCESS:
      return determine_robot_egg_grip(env, f"is_{self.egg_name}_not_in_robot_grip", check_egg=self.egg_name)
    return Status.FAILURE    
  
  def terminate(self, new_status):
    self.logger.debug(f"PutEggInPotAction::terminate to {new_status}")
   



class BoilEggsAction(Behaviour):
  def __init__(self):
    super().__init__("boil_eggs")
    self.conditions =  [
      Condition("is_stove_turn_on", "boil_eggs"),
      Condition("is_pot_in_stove", "boil_eggs"),
      Condition("is_egg1_in_pot", "boil_eggs"),
      Condition("is_egg2_in_pot", "boil_eggs")
    ]
  def setup(self):
    self.logger.debug(f"BoilEggsAction::setup")

  def initialise(self):
    self.logger.debug(f"BoilEggsAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return were_eggs_boiled(act_states, env)
  
  def terminate(self, new_status):
    self.logger.debug(f"BoilEggsAction::terminate to {new_status}")

class TurnOffStoveAction(Behaviour):
  def __init__(self):
    super().__init__("turn_off_stove_better")
    self.conditions =  [
        Condition("is_robot_grip_not_occupied", "turn_off_stove_better"),
        Condition("is_stove_turn_on", "turn_off_stove_better")
    ]
  def setup(self):
    self.logger.debug(f"TurnOffStoveAction::setup")

  def initialise(self):
    self.logger.debug(f"TurnOffStoveAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_stove_state(env, "is_not_stove_turn_on")
  
  def terminate(self, new_status):
    self.logger.debug(f"TurnOffStoveAction::terminate to {new_status}")



class PickEggOutOfPotAction(Behaviour):
  def __init__(self):
    super().__init__("pick_egg_out_of_pot")
    self.conditions = [
      Condition("is_robot_grip_not_occupied", self.name, modify_action=True),
      Condition("is_egg_in_pot", self.name)
    ]
    
  def setup(self):
    self.logger.debug(f"PickEggOutOfPotAction::setup")

  def initialise(self):
    self.logger.debug(f"PickEggOutOfPotAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    self.name = self.conditions[0].action
    self.egg_name = robocasa.demos.singleton.general_information["egg_name"]
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_egg_pot(env, f"is_not_{self.egg_name}_in_pot") == Status.SUCCESS:
      return determine_robot_egg_grip(env, f"is_{self.egg_name}_in_robot_grip", check_egg=self.egg_name)
    return Status.FAILURE
  
  def terminate(self, new_status):
    self.logger.debug(f"PickEggOutOfPotAction::terminate to {new_status}")




class PickPotFromStoveAction(Behaviour):
  def __init__(self):
    super().__init__("pick_pot_from_stove")
    self.conditions = [
        Condition("is_robot_grip_not_occupied", "pick_pot_from_stove"),
        Condition("is_pot_in_stove", "pick_pot_from_stove"),
        Condition("is_egg1_not_in_pot", "pick_pot_from_stove"),
        Condition("is_egg2_not_in_pot", "pick_pot_from_stove")
    ]
  def setup(self):
    self.logger.debug(f"PickPotFromStoveAction::setup")

  def initialise(self):
    self.logger.debug(f"PickPotFromStoveAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_robot_pot_grip(env, "is_pot_in_robot_grip") == Status.SUCCESS:
      return determine_is_pot_in_stove(env, "is_not_pot_in_stove", act_states)[1]
    else:
      return Status.FAILURE
  
  def terminate(self, new_status):
    self.logger.debug(f"PickPotFromStoveAction::terminate to {new_status}")

class SpillWaterAction(Behaviour):
  def __init__(self):
    super().__init__("spill_water")
    self.conditions = [
      Condition("is_pot_in_robot_grip", "spill_water")
    ]
  def setup(self):
    self.logger.debug(f"SpillWaterAction::setup")

  def initialise(self):
    self.logger.debug(f"SpillWaterAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_robot_pot_grip(env, "is_pot_in_robot_grip") == Status.SUCCESS:
      return was_water_spilled(act_states, env)
    else:
      return Status.FAILURE
  
  def terminate(self, new_status):
    self.logger.debug(f"SpillWaterAction::terminate to {new_status}")


class PutPotInCabinetAction(Behaviour):
  def __init__(self):
    super().__init__("put_pot_in_cabinet")
    self.conditions = [
      Condition("is_pot_in_robot_grip", "put_pot_in_cabinet"),
      Condition("is_cabinet_opened", "put_pot_in_cabinet")
    ]
  def setup(self):
    self.logger.debug(f"PutPotInCabinetAction::setup")

  def initialise(self):
    self.logger.debug(f"PutPotInCabinetAction::initialise")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    if determine_pot_cabinet(env, "is_pot_there_in_cabinet") == Status.SUCCESS:
      return determine_robot_pot_grip(env, "is_pot_not_in_robot_grip")
    return Status.FAILURE
  def terminate(self, new_status):
    self.logger.debug(f"PutPotInCabinetAction::terminate to {new_status}")


class OpenCabinetDoorAction(Behaviour):
  def __init__(self):
    super().__init__("open_cabinet_door")
    self.conditions = [
      Condition("is_robot_grip_not_occupied", "open_cabinet_door"),
      Condition("is_cabinet_closed", "open_cabinet_door")
    ]
    
  def setup(self):
    self.logger.debug(f"OpenCabinetDoorAction::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"OpenCabinetDoorAction::initialise {self.name}")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_cabinet_state(env, "is_cabinet_opened", is_action=True)
  
  def terminate(self, new_status):
    self.logger.debug(f"OpenCabinetDoorAction::terminate to {new_status}")
    
    
class CloseDoorCabinetAction(Behaviour):
  def __init__(self):
    super().__init__("close_door_cabinet_better")
    self.conditions = [
    Condition("is_robot_grip_not_occupied", "close_door_cabinet_better"),
    Condition("is_cabinet_opened", "close_door_cabinet_better")]
    

  def setup(self):
    self.logger.debug(f"CloseDoorCabinetAction::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"CloseDoorCabinetAction::initialise {self.name}")

  def update(self):
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_default_playback_dataset
    act_states, final_state, env = get_default_playback_dataset(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.name+"/ep_demo.hdf5", final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
    robocasa.demos.singleton.general_information["not_first_action"] = True
    robocasa.demos.singleton.general_information["previous_state"] = final_state
    robocasa.demos.singleton.general_information["previous_env"] = env
    return determine_cabinet_state(env, "is_cabinet_closed", is_action=True)
  
  def terminate(self, new_status):
    self.logger.debug(f"CloseDoorCabinetAction::terminate to {new_status}")
    

  
def final_condition():
  
  is_okay_1 = robocasa.demos.singleton.world_state["egg1_location"] == robocasa.demos.singleton.goal_state["egg1_location"] and robocasa.demos.singleton.world_state['egg2_location'] == robocasa.demos.singleton.goal_state['egg2_location'] and robocasa.demos.singleton.world_state['is_egg2_boiled'] == robocasa.demos.singleton.goal_state['is_egg2_boiled'] and robocasa.demos.singleton.world_state["is_egg1_boiled"] == robocasa.demos.singleton.goal_state['is_egg1_boiled'] 
  is_okay_2 = robocasa.demos.singleton.world_state['is_cabinet_left_closed'] == robocasa.demos.singleton.goal_state['is_cabinet_left_closed'] and robocasa.demos.singleton.world_state['is_cabinet_right_closed'] == robocasa.demos.singleton.goal_state['is_cabinet_right_closed'] and robocasa.demos.singleton.world_state['is_cabinet_left_opened'] == robocasa.demos.singleton.goal_state['is_cabinet_left_opened'] and robocasa.demos.singleton.world_state["is_cabinet_right_opened"] == robocasa.demos.singleton.goal_state["is_cabinet_right_opened"]
  is_okay_3 = robocasa.demos.singleton.world_state["is_pot_occupied_egg2"] == robocasa.demos.singleton.goal_state["is_pot_occupied_egg2"] and robocasa.demos.singleton.world_state["is_robot_grip_occupied"] == robocasa.demos.singleton.goal_state["is_robot_grip_occupied"] and robocasa.demos.singleton.world_state["is_fridge_closed"] == robocasa.demos.singleton.goal_state["is_fridge_closed"] and robocasa.demos.singleton.world_state["is_fridge_opened"] == robocasa.demos.singleton.goal_state["is_fridge_opened"]
  is_okay_4 = robocasa.demos.singleton.world_state["is_pot_occupied_egg1"] == robocasa.demos.singleton.goal_state["is_pot_occupied_egg1"] and robocasa.demos.singleton.world_state["is_sink_opened"] == robocasa.demos.singleton.goal_state["is_sink_opened"] and robocasa.demos.singleton.world_state["is_sink_positioned"] == robocasa.demos.singleton.goal_state["is_sink_positioned"] and robocasa.demos.singleton.world_state["is_water_boiled"] == robocasa.demos.singleton.goal_state["is_water_boiled"]
  is_okay_5 = robocasa.demos.singleton.world_state["is_water_in_pot"] == robocasa.demos.singleton.goal_state["is_water_in_pot"] and robocasa.demos.singleton.world_state["pot_location"] == robocasa.demos.singleton.goal_state["pot_location"] and robocasa.demos.singleton.world_state["stove.is_active"] == robocasa.demos.singleton.goal_state["stove.is_active"] and robocasa.demos.singleton.world_state["stove.is_occupied"] ==  robocasa.demos.singleton.goal_state["stove.is_occupied"]
  
  if is_okay_1 and is_okay_2 and is_okay_3 and is_okay_4 and is_okay_5:
    return Status.SUCCESS
  else:
    Status.FAILURE 


def find_the_initial_world_state(action):
  robocasa.demos.singleton.world_state["is_egg2_boiled"] = False
  robocasa.demos.singleton.world_state["is_egg1_boiled"] = False

  robocasa.demos.singleton.world_state["is_cabinet_left_opened"] = False
  robocasa.demos.singleton.world_state["is_cabinet_right_opened"] = False

  robocasa.demos.singleton.world_state["is_cabinet_left_closed"] = False
  robocasa.demos.singleton.world_state["is_cabinet_right_closed"] = False

  robocasa.demos.singleton.world_state["is_water_in_pot"] = False
  robocasa.demos.singleton.world_state["is_fridge_opened"] = False
  robocasa.demos.singleton.world_state["is_fridge_closed"] = False

  robocasa.demos.singleton.world_state["is_sink_opened"] = False

  robocasa.demos.singleton.world_state["is_sink_positioned"] = False
  robocasa.demos.singleton.world_state["is_water_boiled"] = False

  robocasa.demos.singleton.world_state["stove.is_active"] = []
  robocasa.demos.singleton.world_state["stove.is_occupied"] = []
    
  robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.FRIDGE
  robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.FRIDGE
  robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.CABINET
  from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_initial_env
  env, initial_state = get_initial_env(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+action+"/ep_demo.hdf5", respective_state=5)
  #global associate_action_with_joint
  robocasa.demos.singleton.associate_action_with_joint = {}
  eggs_num = list(filter(lambda x: x.startswith("egg") and "joint0" in x, env.sim.model.joint_names))
  eggs_num = list(map(lambda x: x.split("_")[0], eggs_num))
  for egg_name in eggs_num:
    determine_egg_pot(env, f"is_{egg_name}_in_pot")
  determine_cabinet_closed(env)
  determine_cabinet_opened(env)
  determine_pot_cabinet(env)
  determine_robot_pot_grip(env)
  determine_robot_egg1_grip(env)
  determine_robot_egg2_grip(env)
  determine_sink_condition(env)
  determine_pot_sink(env)
  determine_pot_position_sink(env)
  determine_fridge_state(env)
  for egg_name in eggs_num:
    determine_egg_fridge(env, f"is_fridge_{egg_name}_there")
    determine_egg_bowl(env, f"is_{egg_name}_in_bowl")
    
  determine_stove_state(env, "", True)
  determine_is_pot_in_stove(env, "", [])
  from copy import deepcopy

  return deepcopy(env), deepcopy(robocasa.demos.singleton.world_state), deepcopy(robocasa.demos.singleton.associate_action_with_joint), deepcopy(initial_state)


class Condition(Behaviour):
  def __init__(self, name, next_action="", modify_action = False):
    self.action = next_action
    super(Condition, self).__init__(name)
    self.modify_action = modify_action
    self.name2 = None
    self.egg_name = None
  def setup(self):
    self.logger.debug(f"Condition::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"Condition::initialise {self.name}")

  def update(self):
    self.logger.debug(f"Condition::update {self.name}")
    from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_initial_env

    

    if self.action != "":
      
      
      if "pick_egg_from_fridge" in self.action:
        if self.modify_action:
          first_egg_name, _ = number_of_eggs(robocasa.demos.singleton.general_information["previous_env"], type="fridge", state=robocasa.demos.singleton.general_information["previous_state"])
          robocasa.demos.singleton.general_information["egg_name"] = first_egg_name

        self.action = self.action.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
        
        if "egg_" in self.name:
          self.name = self.name.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
      elif "pick_egg_from_bowl" in self.action:
        if self.modify_action:
          first_egg_name, _ = number_of_eggs(robocasa.demos.singleton.general_information["previous_env"], type="bowl", state=robocasa.demos.singleton.general_information["previous_state"])
          robocasa.demos.singleton.general_information["egg_name"] = first_egg_name
        self.action = self.action.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
        if "egg_" in self.name:
          self.name = self.name.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
      elif "pick_egg_out_of_pot" in self.action:
        if self.modify_action:
          first_egg_name, _ = number_of_eggs(robocasa.demos.singleton.general_information["previous_env"], type="pot", state=robocasa.demos.singleton.general_information["previous_state"])
          robocasa.demos.singleton.general_information["egg_name"] = first_egg_name
        self.action = self.action.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
        if "egg_" in self.name:
          self.name = self.name.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
      elif "put_egg_in_bowl" in self.action:

        if self.modify_action:
          first_egg_name, _ = number_of_eggs(robocasa.demos.singleton.general_information["previous_env"], type="robot_hand", state=robocasa.demos.singleton.general_information["previous_state"])
          robocasa.demos.singleton.general_information["egg_name"] = first_egg_name
        self.action = self.action.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
        if "egg_" in self.name:
          self.name = self.name.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
      elif "put_egg_in_pot" in self.action:
        if self.modify_action:
          first_egg_name, _ = number_of_eggs(robocasa.demos.singleton.general_information["previous_env"], type="robot_hand", state=robocasa.demos.singleton.general_information["previous_state"])
          robocasa.demos.singleton.general_information["egg_name"] = first_egg_name
        self.action = self.action.replace("egg", robocasa.demos.singleton.general_information["egg_name"])
        if "egg_" in self.name:
          self.name = self.name.replace("egg", robocasa.demos.singleton.general_information["egg_name"])

      env_, _ = get_initial_env(dataset="robocasa/models/assets/demonstrations_private/best_actions/"+self.action+"/ep_demo.hdf5", respective_state=5, final_previous_state=robocasa.demos.singleton.general_information["previous_state"], previous_env=robocasa.demos.singleton.general_information["previous_env"])
      
      if robocasa.demos.singleton.general_information["previous_env"] == None:
        eggs_num = list(filter(lambda x: x.startswith("egg") and "joint0" in x, env_.sim.model.joint_names))
        eggs_num = list(map(lambda x: x.split("_")[0], eggs_num))
        for egg_name in eggs_num:
          determine_egg_pot(env_, f"is_{egg_name}_in_pot")
        determine_cabinet_closed(env_)
        determine_cabinet_opened(env_)
        determine_pot_cabinet(env_)
        determine_robot_pot_grip(env_)
        determine_robot_egg1_grip(env_)
        determine_robot_egg2_grip(env_)
        determine_sink_condition(env_)
        determine_pot_sink(env_)
        determine_pot_position_sink(env_)
        determine_fridge_state(env_)
        for egg_name in eggs_num:
          determine_egg_fridge(env_, f"is_fridge_{egg_name}_there")
          determine_egg_bowl(env_, f"is_{egg_name}_in_bowl")
        determine_stove_state(env_, "", True)
        determine_is_pot_in_stove(env_, "", [])
    
    
    if self.name == "is_cabinet_closed" or self.name == "is_cabinet_opened": 
      return determine_cabinet_state(env_, self.name)
    elif self.name == "is_pot_there_in_cabinet":
      return determine_pot_cabinet(env_, self.name)
    elif self.name == "is_pot_in_sink" or self.name == "is_pot_not_in_sink":
      return determine_pot_sink(env_, self.name)
    elif self.name == "is_pot_in_robot_grip":
      return determine_robot_pot_grip(env_, self.name)
    elif self.name == "is_egg1_in_robot_grip":
      return determine_robot_egg1_grip(env_, self.name)
    elif self.name == "is_egg2_in_robot_grip":
      return determine_robot_egg2_grip(env_, self.name)
    elif self.name == "is_sink_closed" or self.name == "is_sink_opened":
      return determine_sink_condition(env_, self.name)
    elif self.name == "is_pot_in_sink":
      return determine_pot_sink(env_, self.name)
    elif self.name == "is_sink_not_in_pot_position" or self.name == "is_sink_in_pot_position":
      return determine_pot_position_sink(env_, self.name)
    elif self.name == "is_fridge_closed" or self.name == "is_fridge_opened":
      return determine_fridge_state(env_, self.name)
    elif self.name == "is_fridge_egg2_there" or self.name == "is_fridge_egg1_there":
      return determine_egg_fridge(env_, self.name)
    elif self.name == "check_eggs_1":
      return determine_nr_eggs_fridge(env_, self.name)
    elif self.name == "is_not_egg1_in_bowl" or self.name == "is_not_egg2_in_bowl" or self.name=="is_egg2_in_bowl" or self.name=="is_egg1_in_bowl":
      return determine_egg_bowl(env_, self.name)
    elif self.name == "is_not_stove_turn_on" or self.name == "is_stove_turn_on":
      return determine_stove_state(env_, self.name, True)
    elif self.name == "is_not_egg1_in_pot" or self.name == "is_not_egg2_in_pot" or self.name=="is_egg2_in_pot" or self.name=="is_egg1_in_pot":
      return determine_egg_pot(env_, self.name)
    elif self.name == "is_robot_grip_not_occupied":
      if robocasa.demos.singleton.world_state["is_robot_grip_occupied"] == False:
        return Status.SUCCESS
      return Status.FAILURE
    elif self.name == "final_condition":
      return final_condition()
    else:
      sleep(1)
      return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

def custom_initial_state_definition(args=None, time=None):
  import numpy as np
  from robocasa.scripts.dataset_scripts.playback_dataset_hdf5 import get_initial_env
  env, _ = get_initial_env(dataset="robocasa/models/assets/demonstrations_private/best_actions/open_cabinet_door/ep_demo.hdf5", respective_state=5)
  robocasa.demos.singleton.general_information["previous_env"] = env
  initial_state_ = None
  if (args == None or args.time is None) and time is None:
    initial_state_ = np.loadtxt(
    "robocasa/demos/initial_state_file",  
    dtype='float64'        
    )
  elif time is not None:
    initial_state_ = np.loadtxt(
    f"robocasa/demos/initial_state_file_{time}",  
    dtype='float64'        
    )
  else:
    initial_state_ = np.loadtxt(
    f"robocasa/demos/initial_state_file_{args.time}",  
    dtype='float64'        
    )


  robocasa.demos.singleton.general_information["previous_state"] = initial_state_


def get_dict_enum():
  robocasa.demos.singleton.dict_enum = {}
  robocasa.demos.singleton.dict_enum["BOWL"] = Env_Locations.BOWL
  robocasa.demos.singleton.dict_enum["CABINET"] = Env_Locations.CABINET
  robocasa.demos.singleton.dict_enum["FRIDGE"] = Env_Locations.FRIDGE
  robocasa.demos.singleton.dict_enum["COUNTER"] = Env_Locations.COUNTER
  robocasa.demos.singleton.dict_enum["SINK"] = Env_Locations.SINK
  robocasa.demos.singleton.dict_enum["ROBOT_HAND"] = Env_Locations.ROBOT_HAND
  robocasa.demos.singleton.dict_enum["STOVE"] = Env_Locations.STOVE
  robocasa.demos.singleton.dict_enum["POT"] = Env_Locations.POT

  robocasa.demos.singleton.dict_enum["FRONT_LEFT"] = Env_Stove.FRONT_LEFT
  robocasa.demos.singleton.dict_enum["FRONT_RIGHT"] = Env_Stove.FRONT_RIGHT
  robocasa.demos.singleton.dict_enum["REAR_LEFT"] = Env_Stove.REAR_LEFT
  robocasa.demos.singleton.dict_enum["REAR_RIGHT"] = Env_Stove.REAR_RIGHT

def initialize_custom_goal_state(args=None, time=None):
  #f = open('robocasa/demos/goal_state_file', 'r')
  f = None
  if (args is None or args.time is None) and time is None:
    f = open('robocasa/demos/goal_state_file', 'r')
  elif time is not None:
    f = open(f'robocasa/demos/goal_state_file_{time}', 'r')
  else:
    f = open(f"robocasa/demos/goal_state_file_{args.time}", 'r')

  goal_state_text = f.read()
  goal_state_list = goal_state_text.split("\n")[0].split(", ")
  idx_goal_state_element = 0
  get_dict_enum()
  while idx_goal_state_element < len(goal_state_list):
    list_value = goal_state_list[idx_goal_state_element].split(": ")
    key, value = list_value[0], list_value[1]

    if value in robocasa.demos.singleton.dict_enum:
      robocasa.demos.singleton.goal_state[key] = robocasa.demos.singleton.dict_enum[value]
    elif value == "True" or value == "False":
      robocasa.demos.singleton.goal_state[key] = eval(value)
    elif "[" in value:
      value = value[1:-1]
      list_values = []
      while idx_goal_state_element+1 < len(goal_state_list) and ":" not in goal_state_list[idx_goal_state_element+1]:
        list_values.append(robocasa.demos.singleton.dict_enum[goal_state_list[idx_goal_state_element+1]])
        idx_goal_state_element+=1
      
      robocasa.demos.singleton.goal_state[key] = list_values
    idx_goal_state_element+=1

  f.close()

def initialize_custom_world_state(args=None, time=None):
  #f = open('robocasa/demos/world_state_start', 'r+')
  f = None
  if (args is None or args.time is None) and time is None:
    f = open('robocasa/demos/world_state_start', 'r')
  elif time is not None:
    f = open(f'robocasa/demos/world_state_start_{time}', 'r')
  else:
    f = open(f"robocasa/demos/world_state_start_{args.time}", 'r')

  world_state_text = f.read()
  world_state_list = world_state_text.split("\n")
  
  for _world_state in world_state_list[:-1]:
    _world_state_split = _world_state.split(": ", 1)
    key, value = _world_state_split[0], _world_state_split[1]

    if "[" in value:
      value = value[1:-1]
      value_list = value.split(", ")
      list_values = []
      for value_ in value_list:
        if "Env_Locations." in value_:
          list_values.append(robocasa.demos.singleton.dict_enum[value_[len("Env_Locations."):]])
        elif "Env_Stove." in value_:
          value_ = value_[1:]
          value_ = value_.split(":")[0].split(" ")[0]
          list_values.append(robocasa.demos.singleton.dict_enum[value_[len("Env_Stove."):]])          
      robocasa.demos.singleton.world_state[key] = list_values
    elif "Env_Stove." in value:
        robocasa.demos.singleton.world_state[key] = robocasa.demos.singleton.dict_enum[value[len("Env_Stove."):]]
    elif value == "True" or value == "False":
        robocasa.demos.singleton.world_state[key] = eval(value)
    elif "Env_Locations." in value:
      robocasa.demos.singleton.world_state[key] = robocasa.demos.singleton.dict_enum[value[len("Env_Locations."):]]
  
  

def make_bt(args, return_actions=False):

  root = Sequence(name="root", memory=True)
  
  cabinet = Sequence(name="cabinet", memory=True)
  sink = Sequence(name="sink", memory=True)
  stove = Sequence(name="sink", memory=True)
  fridge = Sequence(name="fridge", memory=True)
  return_object = Sequence(name="return_object", memory=True)
  is_this_successful = Sequence(name="is_this_successful", memory=True)

  list_cabinet_actions = []
  from robocasa.environments.kitchen.atomic.custom_kitchen import CustomKitchen
  import numpy as np
  list_of_actions = []

  if args != None:
    get_dict_enum()
    if args.custom_initial_state:
      custom_initial_state_definition(args)
    if args.custom_goal_state:
      initialize_custom_goal_state(args)
    if args.custom_world_state:
      initialize_custom_world_state(args)
      
    if args.action_list:
      list_of_actual_actions = []
      actions_choose = Sequence(name="actions_choose", memory=True)

      for action in args.action_list:
        if action == "open_cabinet":
          open_cabinet_door = OpenCabinetDoorAction()
          list_of_actual_actions.extend(open_cabinet_door.conditions)
          list_of_actual_actions.append(open_cabinet_door)
        elif action == "pick_pot_from_cabinet":
          pick_pot_from_cabinet_better = PickPotFromCabinetAction()
          list_of_actual_actions.extend(pick_pot_from_cabinet_better.conditions)
          list_of_actual_actions.append(pick_pot_from_cabinet_better)
        elif action == "put_pot_in_sink":
          put_pot_in_sink = PutPotInSinkAction()
          list_of_actual_actions.extend(put_pot_in_sink.conditions)
          list_of_actual_actions.append(put_pot_in_sink)
        elif action == "position_faucet_in_pot":
          position_faucet_in_pot = PositionFaucetInPotAction()
          list_of_actual_actions.extend(position_faucet_in_pot.conditions)
          list_of_actual_actions.append(position_faucet_in_pot) 
        elif action == "put_water_in_pot":
          put_water_in_pot = PutWaterInPotAction()
          list_of_actual_actions.extend(put_water_in_pot.conditions)
          list_of_actual_actions.append(put_water_in_pot)       
        elif action == "reposition_faucet":
          reposition_faucet = RepositionFaucetAction()
          list_of_actual_actions.extend(reposition_faucet.conditions)
          list_of_actual_actions.append(reposition_faucet)    
        elif action == "turn_off_sink_better": 
          turn_off_sink_better = TurnOffSinkAction()
          list_of_actual_actions.extend(turn_off_sink_better.conditions)
          list_of_actual_actions.append(turn_off_sink_better)  
        elif action == "pick_pot_from_sink_better":
          pick_pot_from_sink_better = PickPotFromSinkAction()
          list_of_actual_actions.extend(pick_pot_from_sink_better.conditions)
          list_of_actual_actions.append(pick_pot_from_sink_better)  
        elif action == "put_pot_in_stove":
          put_pot_in_stove = PutPotInStoveAction()
          list_of_actual_actions.extend(put_pot_in_stove.conditions)
          list_of_actual_actions.append(put_pot_in_stove)
        elif action == "turn_on_stove":
          turn_on_stove = TurnOnStoveAction()
          list_of_actual_actions.extend(turn_on_stove.conditions)

          list_of_actual_actions.append(turn_on_stove)
        elif action == "turn_on_sink":
          turn_on_sink = TurnOnSinkAction()
          list_of_actual_actions.extend(turn_on_sink.conditions)
          list_of_actual_actions.append(turn_on_sink) 
        elif action == "turn_off_sink":
          turn_off_sink_better = TurnOffSinkAction()
          list_of_actual_actions.extend(turn_off_sink_better.conditions)
          list_of_actual_actions.append(turn_off_sink_better)
        elif action == "boil_water":  
          boil_water = BoilWaterAction()
          list_of_actual_actions.extend(boil_water.conditions)
          list_of_actual_actions.append(boil_water)    
        elif action == "pick_egg_from_bowl":
          pick_egg_from_bowl = PickEggFromBowlAction()
          list_of_actual_actions.extend(pick_egg_from_bowl.conditions)
          list_of_actual_actions.append(pick_egg_from_bowl)
        elif action == "turn_off_stove_better":
          turn_off_stove_better = TurnOffStoveAction()
          list_of_actual_actions.extend(turn_off_stove_better.conditions)
          list_of_actual_actions.append(turn_off_stove_better)    
        elif action == "pick_egg_out_of_pot":
          pick_egg_out_of_pot = PickEggOutOfPotAction()
          list_of_actual_actions.extend(pick_egg_out_of_pot.conditions)
          list_of_actual_actions.append(pick_egg_out_of_pot)   
        
        elif action == "put_egg_in_pot":
          put_egg_in_pot = PutEggInPotAction()
          list_of_actual_actions.extend(put_egg_in_pot.conditions)
          list_of_actual_actions.append(put_egg_in_pot)           
        elif action == "pick_pot_from_stove":
          pick_pot_from_stove = PickPotFromStoveAction()
          list_of_actual_actions.extend(pick_pot_from_stove.conditions)
          list_of_actual_actions.append(pick_pot_from_stove) 
        elif action == "boil_eggs":
          boil_eggs = BoilEggsAction()
          list_of_actual_actions.extend(boil_eggs.conditions)
          list_of_actual_actions.append(boil_eggs)  
        elif action == "spill_water":
          spill_water = SpillWaterAction()
          list_of_actual_actions.extend(spill_water.conditions)
          list_of_actual_actions.append(spill_water)    
        elif action == "close_cabinet":
          close_door_cabinet = CloseDoorCabinetAction()
          list_of_actual_actions.extend(close_door_cabinet.conditions)
          list_of_actual_actions.append(close_door_cabinet)    
        elif action == "put_pot_in_cabinet": 
          put_pot_in_cabinet = PutPotInCabinetAction()
          list_of_actual_actions.extend(put_pot_in_cabinet.conditions)
          list_of_actual_actions.append(put_pot_in_cabinet)   
        elif action == "open_fridge":
          open_fridge_egg = OpenFridgeAction()
          list_of_actual_actions.extend(open_fridge_egg.conditions)
          list_of_actual_actions.append(open_fridge_egg) 
        elif action == "pick_egg_from_fridge":
          pick_egg_from_fridge = PickEggFromFridgeAction()
          list_of_actual_actions.extend(pick_egg_from_fridge.conditions)
          list_of_actual_actions.append(pick_egg_from_fridge)
        elif action == "put_egg_in_bowl":
          put_egg_in_bowl = PutEggInBowlAction()
          list_of_actual_actions.extend(put_egg_in_bowl.conditions)
          list_of_actual_actions.append(put_egg_in_bowl)
        
        elif action == "close_fridge":
          close_fridge = CloseFridgeAction()
          list_of_actual_actions.extend(close_fridge.conditions)
          list_of_actual_actions.append(close_fridge)
        
        
      if return_actions:
        return list_of_actions


      final_condition = Condition("final_condition")
      list_actions_final_condition = [final_condition]

      actions_choose.add_children(
          list_of_actual_actions
      )
      is_this_successful.add_children(
        list_actions_final_condition
      )


      root.add_children(
        [
          actions_choose,
          is_this_successful
          
        ]
      )
      return root

  list_of_actions = [] 
  list_of_actions = [] 
  list_of_actions.append("open_cabinet_door")
  list_of_actions.append("pick_pot_from_cabinet_better")
  list_of_actions.append("put_pot_in_sink")
  list_of_actions.append("turn_on_sink")
  list_of_actions.append("position_faucet_in_pot")
  list_of_actions.append("put_water_in_pot")
  list_of_actions.append("reposition_faucet")
  list_of_actions.append("turn_off_sink_better")
  list_of_actions.append("pick_pot_from_sink_better")
  list_of_actions.append("put_pot_in_stove")
  list_of_actions.append("open_fridge")
  list_of_actions.append("pick_egg2_from_fridge")
  list_of_actions.append("close_fridge")
  list_of_actions.append("put_egg2_in_bowl")
  list_of_actions.append("pick_egg1_from_fridge")
  list_of_actions.append("put_egg1_in_bowl")
  list_of_actions.append("turn_on_stove")
  list_of_actions.append("boil_water")
  list_of_actions.append("pick_egg1_from_bowl")
  list_of_actions.append("put_egg1_in_pot")
  list_of_actions.append("pick_egg2_from_bowl")
  list_of_actions.append("put_egg2_in_pot")
  list_of_actions.append("boil_eggs")
  list_of_actions.append("turn_off_stove_better")
  list_of_actions.append("pick_egg2_out_of_pot")
  list_of_actions.append("pick_egg1_out_of_pot")
  list_of_actions.append("pick_pot_from_stove")
  list_of_actions.append("spill_water")
  list_of_actions.append("put_pot_in_cabinet")
  list_of_actions.append("close_door_cabinet_better")
  
  if return_actions:
    return list_of_actions


#https://www.geeksforgeeks.org/python/how-to-pass-a-list-as-a-command-line-argument-with-argparse/
def list_of_string(arg):
    if len(arg.split(", ")) > 2:
      return list(map(str, arg.split(', ')))
    return list(map(str, arg.split(',')))

def reset():
  robocasa.demos.singleton.general_information = {}
  robocasa.demos.singleton.general_information["previous_state"] = None
  robocasa.demos.singleton.general_information["previous_env"] = None
  robocasa.demos.singleton.general_information["not_first_action"] = False
  robocasa.demos.singleton.associate_action_with_joint = {}
  robocasa.demos.singleton.set_of_initial_states = set()
  robocasa.demos.singleton.goal_state = {}
  robocasa.demos.singleton.goal_state["egg1_location"] = Env_Locations.BOWL
  robocasa.demos.singleton.goal_state['egg2_location'] = Env_Locations.BOWL
  robocasa.demos.singleton.goal_state['is_egg2_boiled'] = True
  robocasa.demos.singleton.goal_state["is_egg1_boiled"] = True 
  robocasa.demos.singleton.goal_state['is_cabinet_left_closed'] = True
  robocasa.demos.singleton.goal_state['is_cabinet_right_closed'] = True
  robocasa.demos.singleton.goal_state['is_cabinet_left_opened'] = False
  robocasa.demos.singleton.goal_state["is_cabinet_right_opened"] = False
  robocasa.demos.singleton.goal_state["is_pot_occupied_egg2"] = False
  robocasa.demos.singleton.goal_state["is_robot_grip_occupied"] = False
  robocasa.demos.singleton.goal_state["is_fridge_closed"] = True
  robocasa.demos.singleton.goal_state["is_fridge_opened"] = False
  robocasa.demos.singleton.goal_state["is_pot_occupied_egg1"] = False
  robocasa.demos.singleton.goal_state["is_sink_opened"] = False
  robocasa.demos.singleton.goal_state["is_sink_positioned"] = False
  robocasa.demos.singleton.goal_state["is_water_boiled"] = False
  robocasa.demos.singleton.goal_state["is_water_in_pot"] = False 
  robocasa.demos.singleton.goal_state["pot_location"] = Env_Locations.CABINET 
  robocasa.demos.singleton.goal_state["stove.is_active"] = [] 
  robocasa.demos.singleton.goal_state["stove.is_occupied"] = []

  robocasa.demos.singleton.world_state = {}
  robocasa.demos.singleton.world_state["is_egg2_boiled"] = False
  robocasa.demos.singleton.world_state["is_egg1_boiled"] = False
  robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False

  robocasa.demos.singleton.world_state["is_cabinet_left_opened"] = False
  robocasa.demos.singleton.world_state["is_cabinet_right_opened"] = False

  robocasa.demos.singleton.world_state["is_cabinet_left_closed"] = False
  robocasa.demos.singleton.world_state["is_cabinet_right_closed"] = False

  robocasa.demos.singleton.world_state["is_water_in_pot"] = False
  robocasa.demos.singleton.world_state["is_fridge_opened"] = False
  robocasa.demos.singleton.world_state["is_fridge_closed"] = False

  robocasa.demos.singleton.world_state["is_sink_opened"] = False

  robocasa.demos.singleton.world_state["is_sink_positioned"] = False
  robocasa.demos.singleton.world_state["is_water_boiled"] = False
  robocasa.demos.singleton.world_state["is_pot_occupied_egg1"] = False
  robocasa.demos.singleton.world_state["is_pot_occupied_egg2"] = False

  robocasa.demos.singleton.world_state["stove.is_active"] = []
  robocasa.demos.singleton.world_state["stove.is_occupied"] = []

  robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.FRIDGE
  robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.FRIDGE
  robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.CABINET

if __name__ == "__main__":

  


  robocasa.demos.singleton.goal_state["egg1_location"] = Env_Locations.BOWL
  robocasa.demos.singleton.goal_state['egg2_location'] = Env_Locations.BOWL
  robocasa.demos.singleton.goal_state['is_egg2_boiled'] = True
  robocasa.demos.singleton.goal_state["is_egg1_boiled"] = True 
  robocasa.demos.singleton.goal_state['is_cabinet_left_closed'] = True
  robocasa.demos.singleton.goal_state['is_cabinet_right_closed'] = True
  robocasa.demos.singleton.goal_state['is_cabinet_left_opened'] = False
  robocasa.demos.singleton.goal_state["is_cabinet_right_opened"] = False
  robocasa.demos.singleton.goal_state["is_pot_occupied_egg2"] = False
  robocasa.demos.singleton.goal_state["is_robot_grip_occupied"] = False
  robocasa.demos.singleton.goal_state["is_fridge_closed"] = True
  robocasa.demos.singleton.goal_state["is_fridge_opened"] = False
  robocasa.demos.singleton.goal_state["is_pot_occupied_egg1"] = False
  robocasa.demos.singleton.goal_state["is_sink_opened"] = False
  robocasa.demos.singleton.goal_state["is_sink_positioned"] = False
  robocasa.demos.singleton.goal_state["is_water_boiled"] = False
  robocasa.demos.singleton.goal_state["is_water_in_pot"] = False 
  robocasa.demos.singleton.goal_state["pot_location"] = Env_Locations.CABINET 
  robocasa.demos.singleton.goal_state["stove.is_active"] = [] 
  robocasa.demos.singleton.goal_state["stove.is_occupied"] = []


  #global robocasa.demos.singleton.general_information
 
  robocasa.demos.singleton.general_information = {}
  robocasa.demos.singleton.general_information["previous_state"] = None
  robocasa.demos.singleton.general_information["not_first_action"] = False
  robocasa.demos.singleton.general_information["previous_env"] = None
  
  robocasa.demos.singleton.world_state["is_egg2_boiled"] = False
  robocasa.demos.singleton.world_state["is_egg1_boiled"] = False
  robocasa.demos.singleton.world_state["is_robot_grip_occupied"] = False

  robocasa.demos.singleton.world_state["is_cabinet_left_opened"] = False
  robocasa.demos.singleton.world_state["is_cabinet_right_opened"] = False

  robocasa.demos.singleton.world_state["is_cabinet_left_closed"] = False
  robocasa.demos.singleton.world_state["is_cabinet_right_closed"] = False

  robocasa.demos.singleton.world_state["is_water_in_pot"] = False
  robocasa.demos.singleton.world_state["is_fridge_opened"] = False
  robocasa.demos.singleton.world_state["is_fridge_closed"] = False

  robocasa.demos.singleton.world_state["is_sink_opened"] = False

  robocasa.demos.singleton.world_state["is_sink_positioned"] = False
  robocasa.demos.singleton.world_state["is_water_boiled"] = False
  robocasa.demos.singleton.world_state["is_pot_occupied_egg1"] = False
  robocasa.demos.singleton.world_state["is_pot_occupied_egg2"] = False

  robocasa.demos.singleton.world_state["stove.is_active"] = []
  robocasa.demos.singleton.world_state["stove.is_occupied"] = []
  robocasa.demos.singleton.set_of_initial_states = set()
  robocasa.demos.singleton.world_state["egg1_location"] = Env_Locations.FRIDGE
  robocasa.demos.singleton.world_state["egg2_location"] = Env_Locations.FRIDGE
  robocasa.demos.singleton.world_state["pot_location"] = Env_Locations.CABINET
  import argparse

  log_tree.level = log_tree.Level.DEBUG
  parser = argparse.ArgumentParser()
  parser.add_argument("--custom_goal_state", action="store_true", help="Have the a custom goal state for the end")
  parser.add_argument("--custom_initial_state", action="store_true", help="Have the a custom initial state for the beginning environment")
  parser.add_argument("--custom_world_state", action="store_true", help="Have the a custom world state at the beginning")
  parser.add_argument("--action_list", type=list_of_string)
  parser.add_argument("--time", type=float, default=None)

  args = parser.parse_args()


  tree = make_bt(args)
  tree.tick_once()
  print("Total time of Atomic Actions=", robocasa.demos.singleton.total_time_action)
  reset()

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

general_information = {}
general_information["previous_state"] = None
general_information["previous_env"] = None
general_information["egg_name"] = None

set_of_initial_states = set()
associate_action_with_joint = {}

goal_state = {}
goal_state["egg1_location"] = Env_Locations.BOWL
goal_state['egg2_location'] = Env_Locations.BOWL
goal_state['is_egg2_boiled'] = True
goal_state["is_egg1_boiled"] = True 
goal_state['is_cabinet_left_closed'] = True
goal_state['is_cabinet_right_closed'] = True
goal_state['is_cabinet_left_opened'] = False
goal_state["is_cabinet_right_opened"] = False
goal_state["is_pot_occupied_egg2"] = False
goal_state["is_robot_grip_occupied"] = False
goal_state["is_fridge_closed"] = True
goal_state["is_fridge_opened"] = False
goal_state["is_pot_occupied_egg1"] = False
goal_state["is_sink_opened"] = False
goal_state["is_sink_positioned"] = False
goal_state["is_water_boiled"] = False
goal_state["is_water_in_pot"] = False 
goal_state["pot_location"] = Env_Locations.CABINET 
goal_state["stove.is_active"] = [] 
goal_state["stove.is_occupied"] = []

world_state = {}
world_state["is_egg2_boiled"] = False
world_state["is_egg1_boiled"] = False
world_state["is_robot_grip_occupied"] = False

world_state["is_cabinet_left_opened"] = False
world_state["is_cabinet_right_opened"] = False

world_state["is_cabinet_left_closed"] = False
world_state["is_cabinet_right_closed"] = False

world_state["is_water_in_pot"] = False
world_state["is_fridge_opened"] = False
world_state["is_fridge_closed"] = False

world_state["is_sink_opened"] = False

world_state["is_sink_positioned"] = False
world_state["is_water_boiled"] = False
world_state["is_pot_occupied_egg1"] = False
world_state["is_pot_occupied_egg2"] = False

world_state["stove.is_active"] = []
world_state["stove.is_occupied"] = []

world_state["egg1_location"] = Env_Locations.FRIDGE
world_state["egg2_location"] = Env_Locations.FRIDGE
world_state["pot_location"] = Env_Locations.CABINET

dict_enum = {}

total_time_action = 0.0
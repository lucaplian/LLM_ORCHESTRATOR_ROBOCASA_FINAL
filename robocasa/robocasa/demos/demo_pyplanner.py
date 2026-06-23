import time
import math
import random
import argparse
import robocasa
from robocasa.demos.demo_pytree import (
    OpenCabinetDoorAction,
    PickPotFromCabinetAction,
    PutPotInSinkAction,
    TurnOnSinkAction,
    PositionFaucetInPotAction,
    PutWaterInPotAction,
    RepositionFaucetAction,
    TurnOffSinkAction,
    PickPotFromSinkAction,
    PutPotInStoveAction,
    OpenFridgeAction,
    PickEgg2FromFridgeAction,
    CloseFridgeAction,
    PutEgg2InBowlAction,
    PickEgg1FromFridgeAction,
    PutEgg1InBowlAction,
    TurnOnStoveAction,
    BoilWaterAction,
    PickEgg1FromBowlAction,
    PutEgg1InPotAction,
    PickEgg2FromBowlAction,
    PutEgg2InPotAction,
    BoilEggsAction,
    TurnOffStoveAction,
    PickEgg2OutOfPotAction,
    PickEgg1OutOfPotAction,
    PickPotFromStoveAction,
    SpillWaterAction,
    PutPotInCabinetAction,
    CloseDoorCabinetAction,
    Condition,
    make_bt
)
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees import logging as log_tree
from google import genai
from openai import OpenAI
import os
from google.genai import types

import copy

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

def is_integer(s):
    return s.isdigit()
def replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name, world_state_1=None, world_state_2=None):       
    #is_egg_in_pot = (world_state_1["pot_location"].name == "ROBOT_HAND" and world_state_2["egg1_location"].name=="POT") or (world_state_2["pot_location"].name == "ROBOT_HAND" and world_state_1["egg1_location"].name=="POT")

    for i in range(1, qpos+1):    
       
        if type(env1.sim.model.get_joint_qpos_addr(f"{joint}")) is tuple:
          
            if world_state_1 != None and world_state_2!=None and (world_state_2["egg1_location"].name == "POT" or world_state_2["egg2_location"].name == "POT")  and world_state_1["pot_location"].name != "STOVE" and ("egg1" in joint or "egg2" in joint):
                initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i] = initial_state1[env1.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i]+initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i]- initial_state2[env2.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i]
            elif world_state_1 != None and world_state_2!=None and (world_state_1["egg1_location"].name == "POT" or world_state_1["egg2_location"].name == "POT") and world_state_1["pot_location"].name != "STOVE" and "pot" in joint :
                if world_state_1["egg1_location"].name == "POT":
                    initial_state1[env1.sim.model.get_joint_qpos_addr(f"egg1_joint0")[0]+i] = initial_state1[env1.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i]+initial_state2[env2.sim.model.get_joint_qpos_addr("egg1_joint0")[0]+i]- initial_state2[env2.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i]
                if  world_state_1["egg2_location"].name == "POT":
                    initial_state1[env1.sim.model.get_joint_qpos_addr(f"egg2_joint0")[0]+i] = initial_state1[env1.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i]+initial_state2[env2.sim.model.get_joint_qpos_addr("egg2_joint0")[0]+i]- initial_state2[env2.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i]
                initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i] = initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i]
            else:
                initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i] = initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i]
            
            
        else:
            initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")+i] = initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")+i]

    for i in range(1, qvel+1):
        if type(env1.sim.model.get_joint_qpos_addr(f"{joint}")) is tuple:
            if world_state_1 != None and world_state_2!=None and (world_state_2["egg1_location"].name == "POT" or world_state_2["egg2_location"].name == "POT")  and world_state_1["pot_location"].name != "STOVE" and ("egg1" in joint or "egg2" in joint):
                initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]] = initial_state1[env1.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]+initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]- initial_state2[env2.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]
            elif world_state_1 != None and world_state_2!=None and (world_state_1["egg1_location"].name == "POT" or world_state_1["egg2_location"].name == "POT") and world_state_2["pot_location"].name != "STOVE" and "pot" in joint:
                if world_state_1["egg1_location"].name == "POT":
                    initial_state1[env1.sim.model.get_joint_qpos_addr(f"egg1_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]] = initial_state1[env1.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]+initial_state2[env2.sim.model.get_joint_qpos_addr("egg1_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]- initial_state2[env2.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]
                if  world_state_1["egg2_location"].name == "POT":
                    initial_state1[env1.sim.model.get_joint_qpos_addr(f"egg2_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]] = initial_state1[env1.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]+initial_state2[env2.sim.model.get_joint_qpos_addr("egg2_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]- initial_state2[env2.sim.model.get_joint_qpos_addr("pot_joint0")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]
                initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]] = initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]
            else:
                initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]] = initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")[0]+i+env2.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]
        else:
            initial_state1[env1.sim.model.get_joint_qpos_addr(f"{joint}")+i+env1.sim.model.get_joint_qvel_addr(f"{last_name}")[1]] = initial_state2[env2.sim.model.get_joint_qpos_addr(f"{joint}")+i+env2.sim.model.get_joint_qvel_addr(f"{last_name}")[1]]

    return initial_state1

def returned_good_parsed_comment(el):
    if "enum" not in str(type(el)):
        return f"{el}"
    else:
        return f"{el.value}"
   
def generate_llm_command_for_prompt(args, actions_spec, actions):

    if args.seed is not None:
        random.seed(args.seed)
        print("it enters here", args.seed)
    
    from robocasa.demos.demo_pytree import find_the_initial_world_state
    actions = actions[:22]
    for i, action in enumerate(actions):
        print(f"Action number {i+1} called {action}")

    idx_action = actions.index("boil_water")
    idx_action_water = actions.index("put_water_in_pot")

    start_time_T1 = time.time()
    
    if args.is_multiple:
        action_1, action_2 = input("Values: ").split()
        
        copy_action_1 = copy.deepcopy(action_1)
        copy_action_2 = copy.deepcopy(action_2)

        if (is_integer(action_1)==False and action_1 not in actions) or (is_integer(action_1) and (int(action_1) > len(actions) or int(action_1) <= 0)):
            action_1 = actions[0]
            
        elif (is_integer(action_1) and (int(action_1)<=len(actions) and int(action_1)>0)):
            action_1 = actions[int(action_1)-1]

        if (is_integer(action_2)==False and action_2 not in actions) or (is_integer(action_2) and (int(action_2) > len(actions) or int(action_2) <= 0)):
            action_2 = actions[0]
        elif (is_integer(action_2) and (int(action_2)<=len(actions) and int(action_2)>0)):
            action_2 = actions[int(action_2)-1]
        
        idx_action1 = actions.index(action_1)
        idx_action2 = actions.index(action_2)

        env1, world_state_1, associate_action_with_joint1, initial_state1 = find_the_initial_world_state(action_1)
        _, second_world_state_1, _, second_initial_state = find_the_initial_world_state(actions[2])

        if int(copy_action_1) > 2:
            initial_state1[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")+1] = second_initial_state[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")+1]
            initial_state1[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")+1] = second_initial_state[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")+1]

            world_state_1["is_cabinet_right_closed"] = second_world_state_1["is_cabinet_right_closed"]
            world_state_1["is_cabinet_left_closed"] = second_world_state_1["is_cabinet_left_closed"]
            world_state_1["is_cabinet_right_opened"] = second_world_state_1["is_cabinet_right_opened"]
            world_state_1["is_cabinet_left_opened"] = second_world_state_1["is_cabinet_right_opened"]

        
        env2, world_state_2, associate_action_with_joint2, initial_state2 = find_the_initial_world_state(action_2)        

        if int(copy_action_2) > 2:
            initial_state2[env2.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")+1] = second_initial_state[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")+1]
            initial_state2[env2.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")+1] = second_initial_state[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")+1]

            world_state_2["is_cabinet_right_closed"] = second_world_state_1["is_cabinet_right_closed"]
            world_state_2["is_cabinet_left_closed"] = second_world_state_1["is_cabinet_left_closed"]
            world_state_2["is_cabinet_right_opened"] = second_world_state_1["is_cabinet_right_opened"]
            world_state_2["is_cabinet_left_opened"] = second_world_state_1["is_cabinet_right_opened"]

        
        copy_initial_state1 = copy.deepcopy(initial_state1)
       
        if action_2 != actions[0]:
            world_state_1["is_cabinet_left_opened"] = True
            world_state_1["is_cabinet_right_opened"] = True
            world_state_1["is_cabinet_left_closed"] = False
            world_state_1["is_cabinet_right_closed"] = False

        if action_1 != actions[0]:
            world_state_2["is_cabinet_left_opened"] = True
            world_state_2["is_cabinet_right_opened"] = True
            world_state_2["is_cabinet_left_closed"] = False
            world_state_2["is_cabinet_right_closed"] = False

        if idx_action1 > idx_action:
            world_state_1["is_water_boiled"] = True
        
        if idx_action2 > idx_action:
            world_state_2["is_water_boiled"] = True

        if idx_action1 > idx_action_water:
            world_state_1["is_water_in_pot"] = True

        if idx_action2 > idx_action_water:
            world_state_2["is_water_in_pot"] = True

        dict_actions = {}
        reverse_dict_action = {}
        value=0

        for idx, key in enumerate(world_state_1.keys()):
            actual_value = -1
            if "_closed" in key and "_".join(key.split("_")[:-1])+"_opened" in reverse_dict_action.keys():
                actual_value = reverse_dict_action["_".join(key.split("_")[:-1])+"_opened"]
                value_list = dict_actions[actual_value]
                value_list.append(key)
                dict_actions[actual_value] = value_list
            elif "_opened" in key and "_".join(key.split("_")[:-1])+"_closed" in reverse_dict_action.keys():
                actual_value = reverse_dict_action["_".join(key.split("_")[:-1])+"_closed"]
                value_list = dict_actions[actual_value]
                value_list.append(key)
                dict_actions[actual_value] = value_list
            elif key == "stove.is_occupied" and "pot_location" in reverse_dict_action.keys():
                actual_value = reverse_dict_action["pot_location"]
                value_list = dict_actions[actual_value]
                value_list.append(key)
                dict_actions[actual_value] = value_list
            elif key == "pot_location" and "stove.is_occupied" in reverse_dict_action.keys():
                actual_value = reverse_dict_action["stove.is_occupied"]
                value_list = dict_actions[actual_value]
                value_list.append(key)
                dict_actions[actual_value] = value_list
            elif key == "egg1_location" and "is_pot_occupied_egg1" in reverse_dict_action.keys():
                actual_value = reverse_dict_action["is_pot_occupied_egg1"]
                value_list = dict_actions[actual_value]
                value_list.append(key)
                dict_actions[actual_value] = value_list
            elif key == "egg2_location" and "is_pot_occupied_egg2" in reverse_dict_action.keys():
                actual_value = reverse_dict_action["is_pot_occupied_egg1"]
                value_list = dict_actions[actual_value]
                value_list.append(key)
                dict_actions[actual_value] = value_list
            else:
                value+=1
                actual_value = value
                dict_actions[actual_value] = [key]
                reverse_dict_action[key] = actual_value

        
        second_state_values = random.sample(range(1, value), value//2)

        copy_value_world_state_1 = copy.deepcopy(world_state_1)
        for keys in second_state_values:
           elements = dict_actions[keys]
           if len(elements) == 1:
                
                if world_state_1[elements[0]] != world_state_2[elements[0]] and elements[0] in associate_action_with_joint1.keys():                    
                    joint, qpos, qvel = associate_action_with_joint1[elements[0]]
                    last_name = env1.sim.model.joint_names[-1]
                    initial_state1 = replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name, world_state_1, world_state_2)
                world_state_1[elements[0]] = world_state_2[elements[0]]  
                
           else:
                
                if world_state_1[dict_actions[keys][0]] == world_state_2[dict_actions[keys][1]] and world_state_1[dict_actions[keys][1]] == world_state_2[dict_actions[keys][0]]:
                    continue

                if dict_actions[keys][0] == "stove.is_occupied" and  dict_actions[keys][1] == "pot_location":
                    if world_state_1[dict_actions[keys][0]] != [] and world_state_2[dict_actions[keys][1]].name !="STOVE":
                        continue
                
                if dict_actions[keys][0] == "pot_location" and  dict_actions[keys][1] == "stove.is_occupied":
                    if world_state_1[dict_actions[keys][1]] != [] and world_state_2[dict_actions[keys][0]].name !="STOVE":
                        continue
                
                if dict_actions[keys][0] == "egg1_location" and  dict_actions[keys][1] == "is_pot_occupied_egg1":
                    if world_state_1[dict_actions[keys][1]] == True and world_state_2[dict_actions[keys][0]].name != "POT":
                        continue
                if dict_actions[keys][0] == "egg2_location" and  dict_actions[keys][1] == "is_pot_occupied_egg2":
                    if world_state_1[dict_actions[keys][1]] == True and world_state_2[dict_actions[keys][0]].name !="POT":
                        continue

                if dict_actions[keys][1] == "egg1_location" and  dict_actions[keys][0] == "is_pot_occupied_egg1":
                    if world_state_1[dict_actions[keys][0]] == True and world_state_2[dict_actions[keys][1]].name != "POT":
                        continue
                if dict_actions[keys][1] == "egg2_location" and  dict_actions[keys][0] == "is_pot_occupied_egg2":
                    if world_state_1[dict_actions[keys][0]] == True and world_state_2[dict_actions[keys][1]].name !="POT":
                        continue

                for kv in dict_actions[keys]:
                    if world_state_1[kv] != world_state_2[kv] and kv in associate_action_with_joint1.keys():
                        joint, qpos, qvel = associate_action_with_joint1[kv]
                        last_name = env1.sim.model.joint_names[-1]                        
                        initial_state1 = replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name, world_state_1, world_state_2)
                    world_state_1[kv] = world_state_2[kv]

        copy_stove_world_state_1 = copy.deepcopy(world_state_1["stove.is_occupied"])
        copy_pot_occupied_world_state_1 = copy.deepcopy(world_state_1["is_pot_occupied_egg1"])
        copy_pot_location_world_state_1 = copy.deepcopy(world_state_1["pot_location"])

        
        if world_state_1["stove.is_occupied"] != [] and world_state_1["pot_location"].name != "STOVE" and world_state_1["egg1_location"] != "STOVE" and world_state_1["egg2_location"].name != "STOVE":
            last_name = env1.sim.model.joint_names[-1]

            joint1, qpos1, qvel1 = associate_action_with_joint1["egg1_location"]

            joint2, qpos2, qvel2 = associate_action_with_joint1["egg2_location"]

            joint3, qpos3, qvel3 = associate_action_with_joint1["pot_location"]


            if world_state_2["egg1_location"] == Env_Locations.STOVE:
                world_state_1["egg1_location"] = world_state_2["egg1_location"]
                initial_state1 = replace_initial_state(env1, env2, qpos1, qvel1, joint1, initial_state1, initial_state2, last_name)
            if world_state_2["egg2_location"] == Env_Locations.STOVE:
                world_state_1["egg2_location"] = world_state_2["egg2_location"]
                initial_state1 = replace_initial_state(env1, env2, qpos2, qvel2, joint2, initial_state1, initial_state2, last_name)
            if world_state_2["pot_location"] == Env_Locations.STOVE:
                world_state_1["pot_location"] = world_state_2["pot_location"]
                initial_state1 = replace_initial_state(env1, env2, qpos3, qvel3, joint3, initial_state1, initial_state2, last_name)


        
        joint, qpos, qvel = associate_action_with_joint1["stove.is_occupied"]
        last_name = env1.sim.model.joint_names[-1]
        if world_state_1["stove.is_occupied"] == [] and world_state_2["stove.is_occupied"] != []:
            if world_state_1["pot_location"].name == "STOVE" or world_state_1["egg1_location"].name == "STOVE" or world_state_1["egg2_location"].name == "STOVE":
                world_state_1["stove.is_occupied"] = world_state_2["stove.is_occupied"]
                initial_state1 = replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name)
        elif world_state_1["stove.is_occupied"] != [] and world_state_2["stove.is_occupied"] == []:
            if world_state_1["pot_location"].name != "STOVE" and world_state_1["egg1_location"].name != "STOVE" and world_state_1["egg2_location"].name != "STOVE":
                world_state_1["stove.is_occupied"] = world_state_2["stove.is_occupied"]
                initial_state1 = replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name)

        elif world_state_1["stove.is_occupied"] == [] and world_state_2["pot_location"].name == "STOVE" and world_state_1["pot_location"].name == world_state_2["pot_location"].name:
            world_state_1["stove.is_occupied"] = world_state_2["stove.is_occupied"]
            initial_state1 = replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name)

        elif world_state_1["stove.is_occupied"] != [] and world_state_1["pot_location"].name != "STOVE" and world_state_1["pot_location"].name == world_state_2["pot_location"].name:
            world_state_1["stove.is_occupied"] = copy_stove_world_state_1
            initial_state1 = replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name)

        if world_state_1["is_pot_occupied_egg1"] == True and world_state_1["pot_location"].name == "ROBOT_HAND":
            if world_state_1["is_pot_occupied_egg1"] == True and world_state_2["is_pot_occupied_egg1"] == False:
                world_state_1["is_pot_occupied_egg1"] = world_state_2["is_pot_occupied_egg1"]
                initial_state1 = replace_initial_state(env1, env2, qpos, qvel, joint, initial_state1, initial_state2, last_name)

        

    else:
        action_1 = input("Value: ")
        
        copy_action_1 = copy.deepcopy(action_1)

        if (is_integer(action_1)==False and action_1 not in actions) or (is_integer(action_1) and (int(action_1) > len(actions) or int(action_1) <= 0)):
            action_1 = actions[0]
            
        elif (is_integer(action_1) and (int(action_1)<=len(actions) and int(action_1)>0)):
            action_1 = actions[int(action_1)-1]

        idx_action1 = actions.index(action_1)
    
        
        env1, world_state_1, associate_action_with_joint1, initial_state1 = find_the_initial_world_state(action_1)
        if int(copy_action_1) > 2:
            _, second_world_state_1, _, second_initial_state = find_the_initial_world_state(actions[2])

            initial_state1[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")+1] = second_initial_state[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_leftdoorhinge")+1]
            initial_state1[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")+1] = second_initial_state[env1.sim.model.get_joint_qpos_addr("hingecabinet_3_main_group_1_rightdoorhinge")+1]

            world_state_1["is_cabinet_right_closed"] = second_world_state_1["is_cabinet_right_closed"]
            world_state_1["is_cabinet_left_closed"] = second_world_state_1["is_cabinet_left_closed"]
            world_state_1["is_cabinet_right_opened"] = second_world_state_1["is_cabinet_right_opened"]
            world_state_1["is_cabinet_left_opened"] = second_world_state_1["is_cabinet_right_opened"]
        if idx_action1 > idx_action:
            world_state_1["is_water_boiled"] = True
        
        if idx_action1 > idx_action_water:
            world_state_1["is_water_in_pot"] = True

    

    
    if world_state_1["egg1_location"].name == "ROBOT_HAND" or world_state_1["egg2_location"].name == "ROBOT_HAND" or world_state_1["pot_location"].name == "ROBOT_HAND":
        world_state_1["is_robot_grip_occupied"] = True
    
    if world_state_1["egg1_location"].name != "ROBOT_HAND" and world_state_1["egg2_location"].name != "ROBOT_HAND" and world_state_1["pot_location"].name != "ROBOT_HAND":
        world_state_1["is_robot_grip_occupied"] = False
    
 
    if world_state_1["egg1_location"].name == "ROBOT_HAND" and world_state_1["pot_location"].name == "ROBOT_HAND":
        world_state_1["egg1_location"] = copy_value_world_state_1["egg1_location"]
        joint_, qpos_, qvel_ = associate_action_with_joint1["egg1_location"]
        initial_state1 = replace_initial_state(env1, env2, qpos_, qvel_, joint_, initial_state1, copy_initial_state1, last_name)
    if world_state_1["egg2_location"].name == "ROBOT_HAND" and world_state_1["pot_location"].name == "ROBOT_HAND":
        world_state_1["egg2_location"] = copy_value_world_state_1["egg2_location"]
        joint_, qpos_, qvel_ = associate_action_with_joint1["egg2_location"]
        initial_state1 = replace_initial_state(env1, env2, qpos_, qvel_, joint_, initial_state1, copy_initial_state1, last_name)



    if world_state_1["is_sink_positioned"] == False and abs(initial_state1[env1.sim.model.get_joint_qpos_addr("sink_island_group_1_spout_joint")+1]) > 0.1:
        initial_state1[env1.sim.model.get_joint_qpos_addr("sink_island_group_1_spout_joint")+1] = 0.0
        env1.sim.data.qpos[env1.sim.model.get_joint_qpos_addr("sink_island_group_1_spout_joint")] = 0.0

    print("world_state_1=", world_state_1)

    world_state_json_format = {}
    for wd_key in world_state_1.keys():
        value = world_state_1[wd_key]
        if value == True or value == False:
            world_state_json_format[wd_key] = value
        elif "'list'" in str(type(value)):
            l_val = []
            for vl in value:
              l_val.append(vl.name)
            world_state_json_format[wd_key] = l_val
        else:
            world_state_json_format[wd_key] = value.name
    import json

    #world_state_json_format = json.dumps(world_state_json_format, indent=2)
    #actions_spec_json = json.dumps(actions_spec, indent=2)


    import pickle
  
    import numpy as np
    np.savetxt(f"robocasa/demos/initial_state_file_{start_time_T1}", initial_state1, delimiter=",")
    with open(f"robocasa/demos/world_state_start_{start_time_T1}", "w") as f:
        for key in world_state_1:
            print(f"{key}: {world_state_1[key]}", file=f)
    LLM_PROMPT = ""
    import re
    first_value = re.split(r'[^\w\s]+', args.prompt)[0]

    import os
    import sys
    script_directory = os.path.dirname(os.path.abspath(sys.argv[0]))
    script_directory=script_directory.replace(" ","\ ")

    if not args.do_it_as_json:
        newline = '\n'
        LLM_PROMPT = "Beginning world state:\n"
        for key in world_state_1.keys():
            if "list" in str(type(world_state_1[key])):
                LLM_PROMPT+=f"- {key}: ["
                for id, el in enumerate(world_state_1[key]):
                    if id != 0:
                        LLM_PROMPT+=", "
                    
                    if "enum" not in str(type(el)):
                        LLM_PROMPT+=f"{el}"
                    else:
                        LLM_PROMPT+=f"{el.value}"

                
                LLM_PROMPT+=f"]{newline}"

            else:

                if "enum" not in str(type(world_state_1[key])):
                    LLM_PROMPT+=f"- {key}: {world_state_1[key]}{newline}"
                else:
                    LLM_PROMPT+=f"- {key}: {world_state_1[key].value}{newline}"
            
        
        LLM_PROMPT += "Available actions:\n"
        for key in actions_spec.keys():
            LLM_PROMPT+=f"- {key}:{newline}"
            for subkeys in actions_spec[key].keys():
                LLM_PROMPT+=f"      * {subkeys}:"
                if "dict" not in str(type(actions_spec[key][subkeys])):
                    LLM_PROMPT+=f" {actions_spec[key][subkeys]}"
                else:
                    for idx, subsubkeys in enumerate(actions_spec[key][subkeys]):
                        if idx == 0:
                            LLM_PROMPT+=f" {subsubkeys}="
                        else:
                            LLM_PROMPT+=f", {subsubkeys}="

                        if "list" in str(type(actions_spec[key][subkeys][subsubkeys])):
                            LLM_PROMPT+="["
                            for id, el in enumerate(actions_spec[key][subkeys][subsubkeys]):
                                if id != 0:
                                    LLM_PROMPT+=", "
                                LLM_PROMPT+=returned_good_parsed_comment(el)
                            LLM_PROMPT+="]"  
                        else:
                            LLM_PROMPT+=returned_good_parsed_comment(actions_spec[key][subkeys][subsubkeys])      
                            
                LLM_PROMPT+=f"{newline}"
            LLM_PROMPT += f'Goal: {first_value}\n'
            LLM_PROMPT += f'Full success condition: {args.prompt}\n'

        LLM_PROMPT_BASIC = copy.deepcopy(LLM_PROMPT)

            
        client_deepseek = OpenAI(api_key="sk-490473d9ca0a4fcfb2bea40766b92bca", base_url="https://api.deepseek.com")

        

        if not args.execute_the_parse_for_the_class:
            
            LLM_PROMPT += f'Full contextualization:\n- {script_directory}/parse_arg.py program is used for parsing the argument and transfering into a file, where it can be later used in the {script_directory}/demo_pytree.py as the goal state dictionary. The responsability of the LLM is to make the argument. You will do it by replacing ARG_GIVEN_BY_LLM with the neccesary command, for example instead of the ARG_GIVEN_BY_LLM, you should write the world_state which should be the final end_goal, (e.g. is_egg2_boiled: False, is_egg1_boiled: False etc., just for you to understand the pattern). In addition, start and end with apostrophes.\n- ^Beginning world state^ represents the conditions from which the environment starts, and you have to consider that when you give the list of actions, which help ensure we reach the goal state\n- LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM are the actions given by you, and they represent the actions after ^Available actions:^, I need only their names, separated by comma (there should be not space after comma, just the next word), not preconditions and effects, only their names.\n'
            LLM_PROMPT += f'- in the ^Full success condition^, each condition is separated by a punctuation(eg. .;:.).\n- for ^Available actions^, each action contains precondities, which need to be fullfiled before the triggering of so action, and and effects, which results by triggering said actions\n'
            LLM_PROMPT += f"Your goal is to devise the correct parameters such as, in the following format (respect this format):\n*LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM=Your answer here.\n*ARG_GIVEN_BY_LLM=Your answer here."
            LLM_PROMPT +=f'ESSENTIAL: Remember that for example actions such as turn_on_sink need to be done before you do position_faucet_in_pot action, and turn_off_sink needs to be done after you do the reposition_faucet action' 

            try_again = 1

            total_tokens = 0
            correction_rounds = 0
            planning_start = time.time()

            all_previous_failed_codes = []
            while try_again == 1:
                try_again = 0
                LLM_PROMPT_COPY = copy.deepcopy(LLM_PROMPT)
                if all_previous_failed_codes != []:
                    LLM_PROMPT_COPY+=f'''
                    These are previous failed code execution, take into consideration this, when you make a new one:
                    {all_previous_failed_codes}
                    '''
                
                response = client_deepseek.chat.completions.create(
                    model="deepseek-v4-pro",
                    temperature=0,
                    messages=[{"role": "user", "content": LLM_PROMPT_COPY}],
                    reasoning_effort="high",
                    extra_body={"thinking": {"type": "enabled"}}
                )

                total_tokens+=  response.usage.total_tokens 
                llm_response = response.choices[0].message.content

                if len(llm_response.split("LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM=")) > 1:
                    if len(llm_response.split("LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM=")[1].split("\n")) > 0:
                        args_given = llm_response.split("LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM=")[1].split("\n")[0]
                    else:
                        try_again=1
                        continue
                else:
                    try_again=1
                    continue

                action_list_response = llm_response.split("LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM=")[1].split("\n")[0]
                if len(llm_response.split("ARG_GIVEN_BY_LLM=")) > 1 and len(llm_response.split("'"))==3:
                    if len(llm_response.split("ARG_GIVEN_BY_LLM=")[1].split("\n")) > 0:
                        args_given = llm_response.split("ARG_GIVEN_BY_LLM=")[1].split("\n")[0]
                    else:
                        try_again=1
                        continue

                else:
                    try_again=1
                    continue
                LLM_PROMPT_SECOND = LLM_PROMPT_BASIC

                LLM_PROMPT_SECOND += f'Full contextualization:\n- ^Beginning world state^ represents the conditions from which the environment starts, and you have to consider that when you verify the list of actions, to help ensure you verify if it reaches the goal state\n- LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM are the actions given which are done to reach the goal state, and they represent the actions after ^Available actions:^, I need to verify if they reach the goal state or not.\n'
                LLM_PROMPT_SECOND += f'This is the list of action give, called LIST_OF_ACTIONS_SEPARED_BY_COMMA_GIVEN_BY_LLM={action_list_response}'
                LLM_PROMPT_SECOND += f'- in the ^Full success condition^, each condition is separated by a punctuation(eg. .;:.).\n- for ^Available actions^, each action contains preconditions, which need to be fullfiled before the triggering of so action, and and effects, which results by triggering said actions.\n'
                LLM_PROMPT_SECOND += f"- REACHED_GOAL_STATE parameter, needs to state in boolean parameter, if the actions taken into consideration, its preconditions and effects, reached their goal state.\n"
                LLM_PROMPT_SECOND += f"Your goal is to devise the correct parameter such as, in the following format (respect this format):\n*REACHED_GOAL_STATE=Your answer here (boolean).\n"
                LLM_PROMPT_SECOND +=f'ESSENTIAL: Remember that, for example, when you verify, actions such as turn_on_sink need to be done before you do the position_faucet_in_pot action, and turn_off_sink needs to be done after you do the reposition_faucet action' 


                response_second = client_deepseek.chat.completions.create(
                    model="deepseek-v4-pro",
                    temperature=0,
                    messages=[{"role": "user", "content": LLM_PROMPT_SECOND}],
                    reasoning_effort="high",
                    extra_body={"thinking": {"type": "enabled"}}
                )
                total_tokens += response_second.usage.total_tokens
                llm_third = response_second.choices[0].message.content
                llm_third = llm_third.split("REACHED_GOAL_STATE")[1].split("'")[0]
                    
        
                if "false" in llm_third.lower():
                    correction_rounds+=1
                    try_again=1
                    all_previous_failed_codes.append(action_list_response)
                    continue

                try:
                    print(f"List of actions: {action_list_response}")
                    planning_time = time.time() - planning_start
                    print(f"Planning time: {planning_time:.2f}s")
                    print(f"Total tokens: {total_tokens}")
                    print(f"Correction rounds: {correction_rounds}")
                    #action_list_response2 = action_list_response.split(",")
                    #print(f"Length of actions: {len(action_list_response2)}")

                    
                    os.system(f"python3 {script_directory}/parse_arg.py {args_given} {start_time_T1}")
                    bt_start = time.time()
                    os.system(f"mjpython -m robocasa.demos.demo_pytree --custom_goal_state --custom_world_state --custom_initial_state --action_list {action_list_response} --time {start_time_T1}")
                    bt_time = time.time() - bt_start
                    print(f"Running time BT: {bt_time:.2f}s")
                    #os.system(f"rm {script_directory}/goal_state_file")
                    #os.system(f"rm {script_directory}/initial_state_file")
                    #os.system(f"rm {script_directory}/world_state_start")
                    return

                except Exception as e:  
                    try_again = 1
        else:
            all_previous_failed_codes = []
            all_previous_non_optimal_codes = []

            IMPORTANT_IMPORTS = f"""import robocasa\nfrom py_trees.behaviour import Behaviour\nfrom py_trees.common import Status\nfrom py_trees.composites import Sequence\nfrom py_trees import logging as log_tree\nfrom robocasa.demos.demo_pytree import (  OpenCabinetDoorAction, PickPotFromCabinetAction, PutPotInSinkAction, TurnOnSinkAction, PositionFaucetInPotAction, PutWaterInPotAction, RepositionFaucetAction, TurnOffSinkAction, PickPotFromSinkAction, PutPotInStoveAction, OpenFridgeAction, PickEgg2FromFridgeAction, CloseFridgeAction, PutEgg2InBowlAction, PickEgg1FromFridgeAction, CloseFridgeAction, PutEgg1InBowlAction, TurnOnStoveAction, BoilWaterAction, PickEgg1FromBowlAction, PutEgg1InPotAction, PickEgg2FromBowlAction, PutEgg2InPotAction, BoilEggsAction, TurnOffStoveAction, PickEgg2OutOfPotAction, PickEgg1OutOfPotAction, PickPotFromStoveAction, SpillWaterAction, PutPotInCabinetAction, CloseDoorCabinetAction, Condition,  reset, get_dict_enum, custom_initial_state_definition, initialize_custom_goal_state, initialize_custom_world_state)\nreset()\nget_dict_enum()\ncustom_initial_state_definition(time={start_time_T1})\ninitialize_custom_goal_state(time={start_time_T1})\ninitialize_custom_world_state(time={start_time_T1})\n
            """
            total_tokens = 0
            correction_rounds = 0
            planning_start = time.time()
            while True:
                
                try_attempts = 0
                FULL_CLASS_CONTEXT = f"""
                Available action nodes (no need to create them, just called them - DO NOT REDEFINE THESE CLASSES, THEY ARE ALREADY IMPLEMENTED):
                - OpenCabinetDoorAction()
                - PickPotFromCabinetAction()
                - PutPotInSinkAction()
                - TurnOnSinkAction()
                - PositionFaucetInPotAction()
                - PutWaterInPotAction()
                - RepositionFaucetAction()
                - TurnOffSinkAction()
                - PickPotFromSinkAction()
                - PutPotInStoveAction()
                - OpenFridgeAction()
                - PickEgg1FromFridgeAction()
                - PickEgg2FromFridgeAction()
                - CloseFridgeAction()
                - PutEgg1InBowlAction()
                - PutEgg2InBowlAction()
                - TurnOnStoveAction()
                - BoilWaterAction()
                - PickEgg1FromBowlAction()
                - PickEgg2FromBowlAction()
                - PutEgg1InPotAction()
                - PutEgg2InPotAction()
                - BoilEggsAction()
                - TurnOffStoveAction()
                - PickEgg1OutOfPotAction()
                - PickEgg2OutOfPotAction()
                - PickPotFromStoveAction()
                - SpillWaterAction()
                - PutPotInCabinetAction()
                - CloseDoorCabinetAction()

                #DO NOT USE import in the beginning for anything
                

                Each actions have the contions defined internally inside the self.
                Template to build a sequence:
                actions_list = []
                action = SampleAction()
                actions_list.extend(action.conditions)
                actions_list.append(action)
                #add more actions if needed
                #CRITICAL: DO NOT WRITE NEW CLASSES OR REDEFINE EXISTING ACTION CLASSES, UNDER NO CIRCUMSTANCE. JUST INSTANTIATE THEM AS SHOWN IN THE TEMPLATE.
                #CRITICAL: DO NOT BUILD ANY CLASSES, JUST build the action_list as shown in the template
                """
                FINAL_FUNCTIONS = """final_condition = Condition("final_condition")\nlist_actions_final_condition = [final_condition]\nlog_tree.level = log_tree.Level.DEBUG\nroot = Sequence(name="root", memory=True)\nactions_choose = Sequence(name="actions_choose", memory=True)\nis_this_successful = Sequence(name="is_this_successful", memory=True)\n\nactions_choose.add_children(\n    actions_list\n)\nis_this_successful.add_children(\n   list_actions_final_condition\n)\n\nroot.add_children(\n    [\n actions_choose,\n   is_this_successful\n    ]\n)\nroot.tick_once()\nprint("Total time of Atomic Actions=", robocasa.demos.singleton.total_time_action)\nreset()\n
                """
                LLM_PROMPT2 = copy.deepcopy(LLM_PROMPT)
                LLM_PROMPT2+=FULL_CLASS_CONTEXT


                
                CONTEXT_WORD = f"""

                Generate the Python code, action classes + sequence, with the actions and the conditions of each action appended to a list, which is going to be used for the sequence.
                IT IS IMPORTANT FOR THE ACTIONS TO LEAD TO THE GOAL STATE, AND TAKE INTO CONSIDERATION THE ITEMS FROM BEGINNING WORLD STATE, NOT THE DEFAULT POSITION.
                ESSENTIAL: Remember that for example actions such as TurnOnSinkAction need to be done before you do PositionFaucetInPotAction, and TurnOffSinkAction needs to be done after you do the RepositionFaucetAction.    
                IMPORTANT: IT IS ESSENTIAL TO TRY TO FIND A GOOD OPTIMAL PATH (WITHOUT REDUNDANCIES) TOWARDS THE GOAL STATE BY TAKING INTO CONSIDERATION THE PRECONDITIONS AND ITS EFFECTS OF EACH ACTION, I WANT TO NOT FAILED IT, AND REACH THE DESTINATION. THINK BEFORE, AND RETRY IN CASE THE ACTION ROUTE IS FAILING, UNTIL YOU GENERATE AN ACTION ROUTE THAT GUARANTEES SUCCESS.
                TO MAKE YOUR GUIDANCE EASIER REMEMBER that a class name coresponds to an action name from the "Available actions" section, to convert a class to an action name you do like this, before each capital if it not the first letter, you need to introduce a "_" line, and them convert all the letters into the small letter, and if you cannot find the action sometimes add "_better"
                """
                LLM_PROMPT2+=CONTEXT_WORD
                if all_previous_failed_codes != []:
                    LLM_PROMPT2+=f'''
                    These are previous failed code execution, take into consideration this, when you make a new one:
                    {all_previous_failed_codes}
                    '''
                

                print("LLM_PROMPT_COPY=", LLM_PROMPT2)
                while True:
                    try:
                        response = client_deepseek.chat.completions.create(
                            model="deepseek-v4-pro",
                            temperature=0,
                            messages=[{"role": "user", "content": LLM_PROMPT2}],
                            reasoning_effort="high",
                            extra_body={"thinking": {"type": "enabled"}}
                        )
                        
                        response_python = response.choices[0].message.content.replace("```python", "").replace("```", "")
                        total_tokens += response.usage.total_tokens

                        break
                    except Exception as e:
                        print(f"Retrying in 5s: {e}")
                        time.sleep(5)
                        try_attempts+=1
                        if try_attempts > 5:
                            break
                if try_attempts <=5:
                    FULL_CLASS_VERIFY = f"""
                    VERIFY IF THE FOLLOWING RESPONSE IS CORRECT:
                    {response}
                    """

                    second_tool_definition = {
                        "name": "verifying_action_plan",
                        "description": "Tries to verify if the python code truly reaches the goal state with SUCCESS or it FAILS",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "reached_goal_state": {
                                    "type": "boolean",
                                    "description": "It describes whether it reached the final goal state AND IT GENERATED Status.SUCCESS"
                                },
                                "list_of_actions_covered": {
                                    "type": "array",
                                    "items": {"type": "string"},
                                    "description": "It covers the list of Classes that robot has covered before reaching his destination or failing to fulfill a certain action's precondition", 
                                },
                                "final_state": {
                                    "type": "object",
                                    "description": "Final state as a dictionary with key-value, after it reached the final destination or was blocked by a certain action's precondition"
                                },
                                "goal_state": {
                                    "type": "object",
                                    "description": "Transform the full_success_condition into a dictionary, remember each condition is separated py punctuation (.,;)."
                                }
                            },
                            "required": ["reached_goal_state", "list_of_actions_covered", "final_state", "goal_state"]
                        }
                    }

                    second_json_arguments = {}
                    second_json_arguments["beginning_world_state"] = world_state_json_format
                    second_json_arguments["available_actions"] = actions_spec
                    second_json_arguments["full_success_condition"] = args.prompt
                    second_json_arguments["python_code"] = response_python
                    context = []
                    #context.append(f'{script_directory}/parse_arg.py program is used for parsing the argument and transfering into a file, where it can be later used in the {script_directory}/demo_pytree.py as the goal state dictionary. The responsability of the LLM is to make the argument. You will do it by replacing ARG_GIVEN_BY_LLM with the neccesary command, for example instead of the ARG_GIVEN_BY_LLM, you should write the world_state which should be the final end_goal, (e.g. is_egg2_boiled: False, is_egg1_boiled: False etc., just for you to understand the pattern). In addition, start and end with apostrophes.')
                    context.append(f'WARNING: YOU MUST VERIFY IN-DEPTH THE "beginning_world_state". Do not assume the a parameter without consulting the "beginning_world_state", and start with that when you verify certain actions.')
                    context.append(f'CRITICAL: CHECK THE LOCATIONS OF OBJECTS (e.g. do not assume that the pot is in the cabinet, or that the pot is in the sink), and take into consideration the states of the environment (e.g. the sink is turned on)')
                    context.append(f'IMPORT: Based on the thinking process try to calculate whether each Action class with its precondition and effects lead to the okay final goal state')    
                    context.append(f'ESSENTIAL: Verify if TunrOnSinkAction is done before you do PositionFaucetInPotAction, and TurnOffSinkAction needs is done after you do the RepositionFaucetAction. If it is not satisfied this fails.')    
                    context.append(f'The key "beginning_world_state" represents the conditions from which the environment starts, and you have to verify the reach_goal_state state reached by taking into consideration the starting point and each action taken and its preconditions.')
                    context.append(f'In the "full_success_condition" key, there is a sentence with each condition which needs to be fulfilled in order to be successful, they are also used when verifying if the "python_code" key was rendered well')
                    context.append(f'For "available_actions" key, each action contains precondities, which need to be fullfiled before the triggering of so action, and and effects, which results by triggering said actions. You need to verifying every precondition, and verify properly if one is the cause of not reaching the final goal state')
                    context.append(f'For "python_code" key, represents the python code, which renders the sequence of action classes, which inside them have preconditions, each action classes done in the code need to reach the goal state with Status.SUCCESS, you need to verify each carefully, taking into consideration the preconditions and their effect on the environment states')
                    second_json_arguments["context"] = context
                    second_json_arguments = json.dumps(second_json_arguments, indent=2)
                    second_input_list = [
                        {"role": "user", "content": f"{second_json_arguments}"}
                    ]
                    
                    while True:
                        try: 
                            response = client_deepseek.chat.completions.create(
                                model="deepseek-v4-pro",
                                tools=[{
                                    "type": "function",
                                    "function": second_tool_definition
                                }],
                                temperature=0,
                                messages=second_input_list,
                                reasoning_effort="high",
                                extra_body={ "thinking": { "type": "enabled" } },
                            )
                            total_tokens += response.usage.total_tokens
                            break
                        except Exception as e:
                            print(f"Retrying in 5s: {e}")
                            time.sleep(5)
                            try_attempts+=1
                            if try_attempts > 5:
                                break
                    if try_attempts > 5:
                        print("FAILED TO FETCH RESPONSE")
                        break
                    
                    if response.choices[0].message.tool_calls[0].function.arguments:
                        function_call_second = response.choices[0].message.tool_calls[0].function
                        function_call_second = json.loads(function_call_second.arguments)
                    
                    if function_call_second["reached_goal_state"] == True:
                        
                        response_python = response_python.replace("```python", "").replace("```", "").strip()
                        script_directory_new = copy.deepcopy(script_directory)
                        script_directory_new = script_directory_new.replace("\\", "")
                        
                        
                        with open(f"{script_directory_new}/parse_classes.py", "w+") as f:
                            f.write(f"{IMPORTANT_IMPORTS}")
                            f.write("\n")
                            f.write(f"{response_python}")
                            f.write("\n")
                            f.write(f"{FINAL_FUNCTIONS}")
                        try: 
                            planning_time = time.time() - planning_start
                            actions_list = list(filter(lambda x: "Action()" in x, response_python.split("\n")))
                            print(f"List of actions: {actions_list}")
                            print(f"Planning time: {planning_time:.2f}s")
                            print(f"Total tokens: {total_tokens}")
                            print(f"Correction rounds: {correction_rounds}")
                            print(f"Length of actions: {len(actions_list)}")
                            
                            goal_state = function_call_second["goal_state"]
                            os.system(f"python3 {script_directory}/parse_arg.py '{goal_state}' '{start_time_T1}'")
                            bt_start = time.time()
                            os.system(f"mjpython -m robocasa.demos.parse_classes")
                            bt_time = time.time() - bt_start
                            print(f"Running time BT: {bt_time:.2f}s")
                            #os.system(f"rm {script_directory_new}/parse_classes.py")
                            #os.system(f"rm {script_directory}/goal_state_file")
                            #os.system(f"rm {script_directory}/initial_state_file")
                            #os.system(f"rm {script_directory}/world_state_start")
                            return
                        except Exception as e:  
                            try_again = 1
                    else:
                        correction_rounds+=1
                        all_previous_failed_codes.append(response_python)

                else:
                    print("FAILED TO FETCH RESPONSE")
    

    else:
        
        all_previous_failed_actions = []
        all_previous_non_optimal_actions = []
        
        try_attempts = 0
        total_tokens = 0
        correction_rounds = 0
        planning_start = time.time()
        while True:
            
            json_arguments = {}
            json_arguments["beginning_world_state"] = world_state_json_format
            json_arguments["available_actions"] = actions_spec
            json_arguments["goal"] = first_value
            json_arguments["full_success_condition"] = args.prompt

            
            if all_previous_failed_actions != []:
                json_arguments["all_previous_failed_actions"] = all_previous_failed_actions
            if all_previous_non_optimal_actions != []:
                json_arguments["all_previous_non_optimal_actions"] = all_previous_non_optimal_actions
            context = []

            
            context.append(f'WARNING: YOU MUST VERIFY IN-DEPTH THE "beginning_world_state". Do not assume the a parameter without consulting the "beginning_world_state", and start with that when you verify certain actions.')
            context.append(f'CRITICAL: CHECK THE LOCATIONS OF OBJECTS (e.g. do not assume that the pot is in the cabinet, or that the pot is in the sink), and take into consideration the states of the environment (e.g. the sink is turned on)')    
            context.append(f'IMPORTANT: Based on the thinking process try to find a good optimal path (without reduncies) towards the goal state taking into consideration the preconditions and effects of each action')    

            context.append(f'ESSENTIAL: Remember that for example actions such as turn_on_sink need to be done before you do position_faucet_in_pot action, and turn_off_sink needs to be done after you do the reposition_faucet action')    
            context.append(f'The key "beginning_world_state" represents the conditions from which the environment starts, and you have to consider that when you give the list of actions, which help ensure we reach the goal state.')
            
            context.append(f'In the "full_success_condition" key, each condition is separated by a punctuation(eg. .;:.). Each sentence needs to be processed in order to produce the final goal_state, and include the False and empty values for other keys. Do not miss any key, ANY.')
            if all_previous_failed_actions != []:
                context.append(f'The key all_previous_failed_actions represent the list of previous of sequence of actions thought by you that have failed previously to reach the full_success_condition, so try to think a different aproach.')
            context.append(f'For "available_actions" key, each action contains precondities, which need to be fullfiled before the triggering of so action, and and effects, which results by triggering said actions')
            if all_previous_non_optimal_actions != []:
                context.append(f'The key all_previous_non_optimal_actions represent the list of previous of sequence of actions thought by you that have suboptimal routes, which means unnecessarry actions that repeat.')
                context.append(f'At the final you have the list of actions that repeat')
            word_added = ""

            
            
            
            if args.optimize or args.hard_optimize:
                word_added = ". Make it as optimal as possible"
                context.append("CRITICAL: FIND THE MOST OPTIMAL ROUTE POSSIBLE, WITH THE LEAST POSSIBLE AMOUNT OF REDUNDANCIES AND DUPLICATES.")
                if args.hard_optimize:
                    context.append("For example, instead of putting the eggs in the bowl after picking the egg from the fridge or any other place, TRY to put them in the pot directly, but make sure that the preconditions and effects are suitable for this.")
            
            json_arguments["context"] = context
            json_arguments = json.dumps(json_arguments, indent=2)
            goal_state_list = ""

            
                
            tool_definition = {
                "name": "execute_action_plan",
                "description": "Generates an action plan for the robot"+word_added,
                "parameters": {
                    "type": "object",
                    "properties": {
                        "action_list": {
                            "type": "array",
                            "items": {"type": "string"},
                            "description": "List of action names"
                        },
                        "goal_state": {
                            "type": "object",
                            "description": "Final goal state as a dictionary with key-value"
                        }
                    },
                    "required": ["action_list", "goal_state"]
                }
            }


            
            input_list = [

                {"role": "user", "content": f"{json_arguments}"}
            ]

            client_deepseek = OpenAI(api_key="sk-490473d9ca0a4fcfb2bea40766b92bca", base_url="https://api.deepseek.com")
         
            try_attempts = 0
            while True:
                print("LLM_PROMPT_COPY=", input_list)
                try:
                    response = client_deepseek.chat.completions.create(
                        model="deepseek-v4-pro",
                        tools=[{
                            "type": "function",
                            "function": tool_definition
                        }],
                        temperature=0,
                        messages=input_list,
                        reasoning_effort=args.thinking_type,
                        extra_body={ "thinking": { "type": args.thinking_activate } },
                    )
                    total_tokens += response.usage.total_tokens

                    break
                except Exception as e:
                    print(f"Retrying in 5s: {e}")
                    time.sleep(5)
                    try_attempts+=1
                    if try_attempts > 5:
                        break
            if try_attempts > 5:
                break


            if response.choices[0].message.tool_calls[0].function.arguments:
                function_call = response.choices[0].message.tool_calls[0].function.arguments
                function_call = json.loads(function_call)
           

            list_act = ""

            for idx, act in enumerate(function_call['action_list']):
                if idx!=0:
                    list_act+=","
                list_act+=act
            second_tool_definition = {}
            
            
            second_tool_definition = {
                "name": "verifying_action_plan",
                "description": "Tries to verify if the action plan truly reaches the goal state or is stopped before by a certain precondition of an action",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "reached_goal_state": {
                            "type": "boolean",
                            "description": "It describes whether it reached the final goal state successfully"
                        },
                        "list_of_actions_covered": {
                            "type": "array",
                            "items": {"type": "string"},
                            "description": "It covers the list of actions that robot has covered before reaching his destination or failing to fulfill a certain action's precondition", 
                        },
                        "final_state": {
                            "type": "object",
                            "description": "Final state as a dictionary with key-value, after it reached the final destination or was blocked by a certain action's precondition"
                        }
                    },
                    "required": ["reached_goal_state", "list_of_actions_covered", "final_state"]
                }
            }

            if args.optimize or args.hard_optimize:
              second_tool_definition["description"] += "In addition, try to verify if the current path contains redundancies (unnecessary duplicate actions)."
              second_tool_definition["parameters"]["properties"]["is_an_optimal_path"] = {
                    "type": "boolean",
                    "description": "It shows whether a list of actions is optimal or not."
              }
              second_tool_definition["parameters"]["properties"]["list_of_non_optimal_actions"] = {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "Show the list of redundant actions that the robot takes, excluding the ones the robot is obliged to take, due to the constraints at the beginning of the environment."
              }
             


            second_json_arguments = {}
            second_json_arguments["beginning_world_state"] = world_state_json_format
            second_json_arguments["available_actions"] = actions_spec
            second_json_arguments["goal_state"] = function_call['goal_state']
            second_json_arguments["list_of_actions"] = function_call['action_list']
            context = []
            context.append(f'WARNING: YOU MUST VERIFY IN-DEPTH THE "beginning_world_state". Do not assume the a parameter without consulting the "beginning_world_state", and start with that when you verify certain actions.')
            context.append(f'CRITICAL: CHECK THE LOCATIONS OF OBJECTS (e.g. do not assume that the pot is in the cabinet, or that the pot is in the sink), and take into consideration the states of the environment (e.g. the sink is turned on)')
            context.append(f'IMPORT: Based on the thinking process try to calculate whether each action with its precondition and effects lead to the okay final goal state')    
            context.append(f'ESSENTIAL: Verify if turn_on_sink is done before you do position_faucet_in_pot action, and turn_off_sink needs is done after you do the reposition_faucet action. If it is not satisfied this fails.')    
            context.append(f'The key "beginning_world_state" represents the conditions from which the environment starts, and you have to verify the reach_goal_state state reached by taking into consideration the starting point and each action taken and its preconditions.')
            context.append(f'In the "goal_state" key, represents the final goal condition needed to be reached out, you need to verify if it reaches this state')
            context.append(f'For "available_actions" key, each action contains precondities, which need to be fullfiled before the triggering of so action, and and effects, which results by triggering said actions. You need to verifying every precondition, and verify properly if one is the cause of not reaching the final goal state')
            context.append(f'For "list_of_actions" key, represents the actions done by the action planner in order to reach the goal state, you need to verify each carefully, taking into consideration the preconditions and their effect on the environment states')
            second_json_arguments["context"] = context
            second_json_arguments = json.dumps(second_json_arguments, indent=2)
            second_input_list = [
                {"role": "user", "content": f"{second_json_arguments}"}
            ]
            
            while True:
                try: 
                    response = client_deepseek.chat.completions.create(
                        model="deepseek-v4-pro",
                        temperature=0,
                        tools=[{
                            "type": "function",
                            "function": second_tool_definition
                        }],
                        messages=second_input_list,
                        reasoning_effort=args.thinking_type,
                        extra_body={ "thinking": { "type": args.thinking_activate } },
                    )
                    total_tokens += response.usage.total_tokens
                    break
                except Exception as e:
                    print(f"Retrying in 5s: {e}")
                    time.sleep(5)
                    try_attempts+=1
                    if try_attempts > 5:
                        break
            
            if try_attempts > 5:
                break
            optimize_ = args.optimize or args.hard_optimize
            if response.choices[0].message.tool_calls[0].function.arguments:
                function_call_second = response.choices[0].message.tool_calls[0].function
                function_call_second = json.loads(function_call_second.arguments)
            
            if function_call_second["reached_goal_state"] == True and (not optimize_ or (optimize_ and function_call_second["is_an_optimal_path"] == True)) :
                goal_state_list = str(function_call_second['final_state'])[1:-1]
                break
            elif function_call_second["reached_goal_state"] == False:
                correction_rounds+=1
                action_list_ = function_call['action_list']
                all_previous_failed_actions.append(f"{action_list_}")
            elif optimize_ and function_call_second["is_an_optimal_path"] == False:
                correction_rounds+=1
                action_list_ = function_call['action_list']
                duplicated_actions = function_call['list_of_non_optimal_actions']
                all_previous_non_optimal_actions.append(f"{action_list_}. These actions were duplicated in particular {duplicated_actions}")
            
        if try_attempts > 5:
            print("FAILED TO FETCH THE LLM's API")
            return
        
        planning_time = time.time() - planning_start
        print(f"List of actions: {list_act}")
        #list_act = list_act.split(",")
        print(f"Planning time: {planning_time:.2f}s")
        print(f"Total tokens: {total_tokens}")
        print(f"Correction rounds: {correction_rounds}")
        #print(f"Length of actions: {len(list_act)}")
        os.system(f"python3 {script_directory}/parse_arg.py '{goal_state_list}' '{start_time_T1}'")
        bt_start = time.time()
        os.system(f"mjpython -m robocasa.demos.demo_pytree --custom_goal_state --custom_world_state --custom_initial_state --action_list {list_act} --time {start_time_T1}")
        bt_time = time.time() - bt_start
        print(f"Running time BT: {bt_time:.2f}s")
        
        return
        #os.system(f"rm {script_directory}/goal_state_file")
        #os.system(f"rm {script_directory}/initial_state_file")
        #os.system(f"rm {script_directory}/world_state_start")

if __name__ == "__main__":
    actions_spec = {
        "open_cabinet": {
            "preconditions": {
                "is_cabinet_left_opened": False,
                "is_cabinet_right_opened": False,
                "is_cabinet_left_closed": True,
                "is_cabinet_right_closed": True,
                "is_robot_grip_occupied": False

            },
            "effects": {
                "is_cabinet_left_opened": True,
                "is_cabinet_right_opened": True,
                "is_cabinet_left_closed": False,
                "is_cabinet_right_closed": False,
            },
            "description": "opens both the cabinet doors"
        },
        "pick_pot_from_cabinet": {
            "preconditions": {
                "is_cabinet_left_opened": True,
                "is_cabinet_right_opened": True,
                "is_cabinet_left_closed": False,
                "is_cabinet_right_closed": False,

                "pot_location": "CABINET",
                "is_robot_grip_occupied": False


            },
            "effects": {
                "pot_location": "ROBOT_HAND",
                "is_robot_grip_occupied": True

            },
            "description": "pick the pot from the cabinet"
        },
        "put_pot_in_sink": {
            "preconditions": {
                "pot_location": "ROBOT_HAND",
                "is_robot_grip_occupied": True

            },
            "effects": {
                "pot_location": "SINK",
                "is_robot_grip_occupied": False,

            },
            "description": "it positions the pot inside the sink"
        },
        "turn_on_sink": {
            "preconditions": {
                "is_sink_opened": False,
                "is_robot_grip_occupied": False
            },
            "effects": {
                "is_sink_opened": True
            },
            "description": "turns on the sink sprayer"
        },
        "position_faucet_in_pot": {
            "preconditions": {
                "pot_location": "SINK",
                "is_sink_positioned": False,
                "is_robot_grip_occupied": False

            },
            "effects": {
            "is_sink_positioned": True,
            },
            "description": "it makes the sink faucet alligned with the pot"
        },
        "put_water_in_pot": {
            "preconditions": {
                "pot_location": "SINK",
                "is_sink_positioned": True,
                "is_robot_grip_occupied": False,
                "is_pot_occupied_egg1": False,
                "is_pot_occupied_egg2": False,

            },
            "effects": {
            "is_water_in_pot": True
            },
            "description": "it fills the pot with water"
        },


        "reposition_faucet": {
            "preconditions": {
                "is_sink_positioned": True,
                "is_robot_grip_occupied": False
            },
            "effects": {
            "is_sink_positioned": False,
            },
            "description": "it positions the faucet back to its original position"
        },

        "turn_off_sink_better": {
            "preconditions": {
                "is_sink_opened": True,
                "is_robot_grip_occupied": False

            },
            "effects": {
            "is_sink_opened": False
            },
            "description": "it turns off the sink sprayer"
        },

        "pick_pot_from_sink_better": {
            "preconditions": {
                "pot_location": "SINK",
                "is_robot_grip_occupied": False

            },
            "effects": {
            "pot_location": "ROBOT_HAND",
            "is_robot_grip_occupied": True
            },
            "description": "it turns off the sink sprayer"
        },

        "put_pot_in_stove": {
            "preconditions": {
                "stove.is_occupied": [],
                "pot_location": "ROBOT_HAND",
                "is_robot_grip_occupied": True

            },
            "effects": {
            "stove.is_occupied": ["FRONT_LEFT"],
            "pot_location": "STOVE",
            "is_robot_grip_occupied": False
            },
            "description": "it puts the pot in the first eye of the stove locating in the front left"
        },

        "open_fridge": {
            "preconditions": {
                "is_fridge_closed": True,
                "is_fridge_opened": False,
                "is_robot_grip_occupied": False

            },
            "effects": {
                "is_fridge_opened": True,
                "is_fridge_closed": False,
            },
            "description": "opens the fridge"
        },
        "pick_egg2_from_fridge": {
            "preconditions": {
                "is_fridge_closed": False,
                "is_fridge_opened": True,
                "egg2_location":  "FRIDGE",
                "is_robot_grip_occupied": False
            },
            "effects": {
                "egg2_location": "ROBOT_HAND",
                "is_robot_grip_occupied": True
            },
            "description": "picks the egg2 from the fridge"
        },
        
        "put_egg2_in_bowl": {
            "preconditions": {
                "egg2_location": "ROBOT_HAND",
                "is_robot_grip_occupied": True,
            },
            "effects": {
                "egg2_location": "BOWL",
                "is_robot_grip_occupied": False,

            },
            "description": "puts the egg2 in the bowl"
        },
        
        
        "pick_egg1_from_fridge": {
            "preconditions": {
                "is_fridge_opened": True,
                "is_fridge_closed": False,
                "egg1_location": "FRIDGE",
                "is_robot_grip_occupied": False
            },
            "effects": {
                "is_robot_grip_occupied": True,
                "egg1_location": "ROBOT_HAND",
            },
            "description": "picks the egg1 from the fridge"
        },

        "close_fridge": {
            "preconditions": {
                "is_fridge_opened": True,
                "is_fridge_closed": False,
            },
            "effects": {
                "is_fridge_opened": False,
                "is_fridge_closed": True,

            },
            "description": "closes the fridge with the egg1 inside its gripper"

        },
        "put_egg1_in_bowl": {
            "preconditions": {
                "egg1_location": "ROBOT_HAND",
                "is_robot_grip_occupied":  True
            },
            "effects": {
                "egg1_location": "BOWL",
                "is_robot_grip_occupied":  False
            },
            "description": "puts the egg1 in the bowl"

        },
        "turn_on_stove": {
            "preconditions": {
                "stove.is_active": [],
                "is_robot_grip_occupied":  False,
            },
            "effects": {
                "stove.is_active": ["FRONT_LEFT"],

            },
            "description": "turns on the stove"

        },
        "boil_water": {
            "preconditions": {
                "stove.is_active": ["FRONT_LEFT"],
                "stove.is_occupied": ["FRONT_LEFT"],
                "is_pot_occupied_egg1": False,
                "is_pot_occupied_egg2": False,
                "is_water_boiled": False,
                "pot_location": "STOVE",
                "is_robot_grip_occupied":  False
            },
            "effects": {
                "is_water_boiled": True,
            },
            "description": "boils the water without any egg inside"

        },
        "pick_egg1_from_bowl": {
            "preconditions": {
                "egg1_location": "BOWL",
                "is_robot_grip_occupied":  False
            },
            "effects": {
                "is_robot_grip_occupied":  True,
                "egg1_location": "ROBOT_HAND",
            },
            "description": "it picks the egg1 from the bowl"
        },

        "put_egg1_in_pot": {
            "preconditions": {
                "egg1_location": "ROBOT_HAND",
                "is_robot_grip_occupied":  True,
                "is_pot_occupied_egg1": False,
                "is_water_boiled": True
            },
            "effects": {
                "is_robot_grip_occupied":  False,
                "egg1_location": "POT",
                "is_pot_occupied_egg1": True


            },
            "description": "it puts the egg1 inside the pot"
        },
        "pick_egg2_from_bowl": {
            "preconditions": {
                "egg2_location": "BOWL",
                "is_robot_grip_occupied":  False
            },
            "effects": {
                "is_robot_grip_occupied":  True,
                "egg2_location": "ROBOT_HAND",
            },
            "description": "it picks the egg2 from the bowl"

        },

        "put_egg2_in_pot": {
            "preconditions": {
                "egg2_location": "ROBOT_HAND",
                "is_robot_grip_occupied":  True,
                "is_pot_occupied_egg2": False,
                "is_water_boiled": True
            },
            "effects": {
                "is_robot_grip_occupied":  False,
                "egg2_location": "POT",
                "is_pot_occupied_egg2": True
            },
            "description": "it puts the egg2 inside the pot"

        },
        "boil_eggs": {
            "preconditions": {
                "stove.is_active": ["FRONT_LEFT"],
                "stove.is_occupied": ["FRONT_LEFT"],
                "egg1_location": "POT",
                "egg2_location": "POT",
                "is_pot_occupied_egg2": True,
                "is_pot_occupied_egg1": True,
                "is_water_boiled": True,
                "is_egg2_boiled":False,
                "is_egg1_boiled":False,
                "pot_location": "STOVE",
                "is_robot_grip_occupied":  False,

            },
            "effects": {
                "is_egg2_boiled":True,
                "is_egg1_boiled":True
            },
            "description": "boils the eggs"

        },
        "turn_off_stove_better": {
            "preconditions": {
                "stove.is_active": ["FRONT_LEFT"],
                "is_robot_grip_occupied":  False,

            },
            "effects": {
                "stove.is_active": [],
            },
            "description": "turns off the stove in the first eye, which is located in the front left"
        },
        "pick_egg2_out_of_pot": {
            "preconditions": {
                "egg2_location": "POT",
                "is_robot_grip_occupied":  False,
                "is_pot_occupied_egg2": True,
                "pot_location": "STOVE",
                "stove.is_occupied": ["FRONT_LEFT"],
            },
            "effects": {
                "is_robot_grip_occupied":  True,
                "egg2_location": "ROBOT_HAND",
                "is_pot_occupied_egg2": False

            },
            "description": "picks the egg2 out of the pot"

        },
        "pick_egg1_out_of_pot": {
            "preconditions": {
                "egg1_location": "POT",
                "is_robot_grip_occupied":  False,
                "is_pot_occupied_egg1": True,
                "pot_location": "STOVE",
                "stove.is_occupied": ["FRONT_LEFT"],
            },
            "effects": {
                "is_robot_grip_occupied":  True,
                "egg1_location": "ROBOT_HAND",
                "is_pot_occupied_egg1": False
            },
            "description": "picks the egg1 out of the pot"

        },
        "pick_pot_from_stove":{
            "preconditions": {
                "pot_location": "STOVE",
                "stove.is_occupied": ["FRONT_LEFT"],
                "is_pot_occupied_egg1": False,
                "is_pot_occupied_egg2": False,
                "is_robot_grip_occupied":  False
            },
            "effects": {
                "pot_location": "ROBOT_HAND",
                "stove.is_occupied": [],
                "is_robot_grip_occupied":  True
            },
            "description":"it picks the pot from the first eye of the stove, which is in the front left"
        },
        "spill_water":{
        "preconditions": {
                "pot_location": "ROBOT_HAND",
                "is_water_in_pot":True,
                "is_robot_grip_occupied":  True

                
            },
            "effects": {
                "is_water_in_pot": False,
                "is_water_boiled": False,
            },
            "descriptions": "it mimics the action of spilling the water, by turning the pot upside down above the sink"
        }, "put_pot_in_cabinet": {
            "preconditions": {
                "is_cabinet_left_opened": True,
                "is_cabinet_right_opened": True,
                "is_cabinet_left_closed": False,
                "is_cabinet_right_closed": False,
                "pot_location": "ROBOT_HAND",
                "is_robot_grip_occupied":  True

            },
            "effects": {
                "pot_location": "CABINET",
                "is_robot_grip_occupied":  False
            },
            "description":"puts the pot inside the cabinet"
        },
        
        "close_cabinet": {
            "preconditions": {
                "is_cabinet_left_opened": True,
                "is_cabinet_right_opened": True,
                "is_cabinet_left_closed": False,
                "is_cabinet_right_closed": False,
                "is_robot_grip_occupied":  False
            },
            "effects": {
                "is_cabinet_left_opened": False,
                "is_cabinet_right_opened": False,
                "is_cabinet_left_closed": True,
                "is_cabinet_right_closed": True,
            },
            "description": "closes both the doors of the cabinet"

        }
    }
    parser = argparse.ArgumentParser()
    actions = make_bt(None, return_actions=True)
    
    parser.add_argument("--is_multiple", action="store_true", help="choose two actions or not")
    parser.add_argument("--do_it_as_json", action="store_true", help="parse it as json")

    parser.add_argument("--thinking_type",  type=str, default="high")
    parser.add_argument("--thinking_activate",  type=str, default="enabled")

    parser.add_argument("--program_class_code", action="store_true", help="program the classes directly using code directly")

    parser.add_argument("--execute_the_parse_for_the_class", action="store_true", help="program the classes directly using code directly")
    parser.add_argument("--execute_the_parse_as_action", action="store_true", help="program the classes directly using code directly")
    parser.add_argument("--seed", type=int, default=None)



    parser.add_argument(
        "--prompt",
        type=str,
        default = "Boils two eggs. Put the eggs in the bowl. Spill the water inside the pot. Put the pot in the cabinet. Close the cabinet. Turn off the stove. Close the fridge. Turn off the sink."
    )

    
    parser.add_argument("--optimize", action="store_true", help="LLM needs to find the most optimal way possible")
    parser.add_argument("--hard_optimize", action="store_true", help="LLM needs to find the most optimal way possible")

    args = parser.parse_args()
    generate_llm_command_for_prompt(args, actions_spec, actions)





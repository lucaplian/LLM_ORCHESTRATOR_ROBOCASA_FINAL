import argparse
import json
import time
from collections import OrderedDict
import os
import robocasa
import datetime
import json
import robosuite
from robosuite.controllers import load_composite_controller_config
from robosuite.wrappers import VisualizationWrapper
from termcolor import colored
from robosuite.wrappers import DataCollectionWrapper, VisualizationWrapper


import robocasa.macros as macros
from robocasa.scripts.collect_demos import collect_human_trajectory, gather_demonstrations_as_hdf5
from robocasa.wrappers.enclosing_wall_render_wrapper import (
    EnclosingWallRenderWrapper,
    install_enclosing_wall_hotkeys,
)
from robocasa.environments.kitchen.atomic.custom_kitchen import CustomKitchen


def choose_option(
    options, option_name, show_keys=False, default=None, default_message=None
):
    """
    Prints out environment options, and returns the selected env_name choice

    Returns:
        str: Chosen environment name
    """
    # get the list of all tasks

    if default is None:
        default = options[0]

    if default_message is None:
        default_message = default

    # Select environment to run
    print("Here is a list of {}s:\n".format(option_name))

    for i, (k, v) in enumerate(options.items()):
        if show_keys:
            print("[{}] {}: {}".format(i, k, v))
        else:
            print("[{}] {}".format(i, v))
    print()
    try:
        s = input(
            "Choose an option 0 to {}, or any other key for default ({}): ".format(
                len(options) - 1,
                default_message,
            )
        )
        # parse input into a number within range
        k = min(max(int(s), 0), len(options) - 1)
        choice = list(options.keys())[k]
    except:
        if default is None:
            choice = options[0]
        else:
            choice = default
        print("Use {} by default.\n".format(choice))

    # Return the chosen environment name
    return choice


if __name__ == "__main__":
    # Arguments

    parser = argparse.ArgumentParser()
    parser.add_argument("--task", type=str, help="task (choose among 365 tasks)")
    parser.add_argument(
        "--layout", type=int, help="kitchen layout (choose number 1-60)"
    )
    parser.add_argument("--style", type=int, help="kitchen style (choose number 1-60)")
    parser.add_argument(
        "--device",
        type=str,
        default="keyboard",
        choices=["keyboard", "spacemouse"],
        help="Teleop device (default: keyboard)",
    )

    parser.add_argument(
        "--directory",
        type=str,
        default=os.path.join(robocasa.models.assets_root, "demonstrations_private"),
    )


    args = parser.parse_args()

    tasks = OrderedDict(
        [
            ("PickPlaceCounterToCabinet", "pick and place from counter to cabinet"),
            ("PickPlaceCounterToSink", "pick and place from counter to sink"),
            ("PickPlaceMicrowaveToCounter", "pick and place from microwave to counter"),
            ("PickPlaceStoveToCounter", "pick and place from stove to counter"),
            ("OpenSingleDoor", "open cabinet or microwave door"),
            ("CloseDrawer", "close drawer"),
            ("TurnOnMicrowave", "turn on microwave"),
            ("TurnOnSinkFaucet", "turn on sink faucet"),
            ("TurnOnStove", "turn on stove"),
            ("ArrangeVegetables", "arrange vegetables on a cutting board"),
            ("MicrowaveThawing", "place frozen food in microwave for thawing"),
            ("RestockPantry", "restock cans in pantry"),
            ("PreSoakPan", "prepare pan for washing"),
            ("PrepareCoffee", "make coffee"),
            ("CustomKitchen", "custom kitchen")
        ]
    )

    if args.task is None:
        args.task = choose_option(
            tasks, "task", default="PickPlaceCounterToCabinet", show_keys=True
        )

    

    # Create argument configuration
    config = {
        "env_name": args.task,
        "robots": "PandaOmron",
        "controller_configs": load_composite_controller_config(robot="PandaOmron"),
        "layout_ids": [31],
        "style_ids": [53],
        "translucent_robot": True,
    }



    args.renderer = "mjviewer"



    print(colored(f"Initializing environment...", "yellow"))
    
    if args.task == "CustomKitchen":
        env = CustomKitchen(
        robots="PandaOmron",
        has_renderer=True,
        has_offscreen_renderer=False,
        render_camera="robot0_frontview",
        ignore_done=True,
        use_camera_obs=False,
        control_freq=20,
        renderer=args.renderer,
        seed=0
        )

        
    else:  
        env = robosuite.make(
            **config,
            has_renderer=True,
            has_offscreen_renderer=False,
            render_camera="robot0_frontview",
            ignore_done=True,
            use_camera_obs=False,
            control_freq=20,
            renderer=args.renderer,
        )
    env_name = "Kitchen"
    # Wrap this with visualization wrapper
    env.ep_directory = os.path.join(robocasa.models.assets_root, "demonstrations_private")
    env = VisualizationWrapper(env)
    env = EnclosingWallRenderWrapper(env, alpha=0.1, enabled=False)



    successful_episodes = []
    directory_path = env.ep_directory
    t_now = time.time()
    time_str = datetime.datetime.fromtimestamp(t_now).strftime("%Y-%m-%d-%H-%M-%S")
    time_str = f"{time_str}_{env_name}"
    
    demo_dir = os.path.join(directory_path, time_str)
    os.makedirs(demo_dir)

    all_eps_directory = os.path.join(demo_dir, "episodes")
    env = DataCollectionWrapper(env, all_eps_directory, use_env_xml_for_reset=True)
    install_enclosing_wall_hotkeys(env)


    if config["layout_ids"] == None and env.get_ep_meta()["layout_id"]:
        config["layout_ids"] = [env.get_ep_meta()["layout_id"]]

    if config["style_ids"] == None and env.get_ep_meta()["style_id"]:
        config["style_ids"] = [env.get_ep_meta()["style_id"]]
    # Grab reference to controller config and convert it to json-encoded string
    env_info = json.dumps(config)

    # initialize device
    device = args.device
    if device == "dualsense":
        from robosuite.devices import DualSense

        device = DualSense(
            env=env,
            pos_sensitivity=4.0,
            rot_sensitivity=4.0,
            vendor_id=macros.DUALSENSE_VENDOR_ID,
            product_id=macros.DUALSENSE_PRODUCT_ID,
        )
        
    if device == "keyboard":
        from robosuite.devices import Keyboard

        device = Keyboard(env=env, pos_sensitivity=4.0, rot_sensitivity=4.0)
    elif device == "spacemouse":
        from robosuite.devices import SpaceMouse

        device = SpaceMouse(
            env=env,
            pos_sensitivity=4.0,
            rot_sensitivity=4.0,
            vendor_id=macros.SPACEMOUSE_VENDOR_ID,
            product_id=macros.SPACEMOUSE_PRODUCT_ID,
        )
    else:
        raise ValueError
    
    # collect demonstrations

    


    ep_directory, ep_subdirectories, discard_traj = collect_human_trajectory(
        env,
        device,
        "right",
        "single-arm-opposed",
        mirror_actions=True,
        render=(args.renderer != "mjviewer"),
        max_fr=30,
    )



    if ep_directory !=None:
        with open(os.path.join(ep_directory, "ep_stats.json"), "w") as file:
            json.dump({"success": not discard_traj}, file)
    
        with open(os.path.join(ep_directory, "env_info.json"), "w") as file:
            json.dump(env_info, file)

        if not discard_traj:
            successful_episodes.append(ep_directory.split("/")[-1])

    
        hdf5_paths, grp, state_paths, states, actions = gather_demonstrations_as_hdf5(
            all_eps_directory,
            ep_directory,
            ep_subdirectories,
            env_info,
            successful_episodes=[ep_directory.split("/")[-1]],
            out_name="ep_demo.hdf5",
            expanded = True
        )
       

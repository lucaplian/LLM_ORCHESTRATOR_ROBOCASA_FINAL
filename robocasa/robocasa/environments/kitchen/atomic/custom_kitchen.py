from robocasa.environments.kitchen.kitchen import *


class CustomKitchen(Kitchen):
    """
    Class encapsulating the atomic microwave press button tasks.

    Args:
        behavior (str): "turn_on" or "turn_off". Used to define the desired
            microwave manipulation behavior for the task
    """

    def __init__(self, *args, **kwargs):

        self.behavior = ""
        super().__init__(*args, **kwargs)
    def set_cameras(self):
        """Override to set custom zoomed out camera"""
        # Call parent to initialize cameras
        super().set_cameras()
        
        # Modify camera configs to zoom out
        from scipy.spatial.transform import Rotation
        
        for cam_name, _ in self._cam_configs.items():
            if "agentview" in cam_name:
                if "camera_attribs" not in self._cam_configs[cam_name]:
                    self._cam_configs[cam_name]["camera_attribs"] = {}
                self._cam_configs[cam_name]["camera_attribs"]["fovy"] = "120"
        
       
    def _setup_kitchen_references(self):
        super()._setup_kitchen_references()
        self.counter = self.get_fixture(FixtureType.COUNTER)
        self.fridge = self.get_fixture(FixtureType.FRIDGE, ref=self.counter)
        self.sink = self.get_fixture(FixtureType.SINK)
        self.cabinet = self.get_fixture(FixtureType.CABINET)
        self.stove = self.get_fixture(FixtureType.STOVE, ref=self.counter)
        self.init_robot_base_pos = self.fridge
       
        

    def get_ep_meta(self):
        """
        Get the episode metadata for the microwave tasks.
        This includes the language description of the task.
        """
        ep_meta = super().get_ep_meta()
        if self.behavior == "turn_on":
            ep_meta["lang"] = "press the start button on the microwave"
        elif self.behavior == "turn_off":
            ep_meta["lang"] = "press the stop button on the microwave"
        return ep_meta

    def _get_obj_cfgs(self):
        """
        Get the object configurations for the microwave tasks. This includes the object placement configurations.
        Place the object inside the microwave and on top of another container object inside the microwave

        Returns:
            list: List of object configurations.
        """        

        cfgs = []
        

        cfgs.append(
            dict(
                name="bowl1",
                obj_groups="bowl",
                mjcf_path = None,

                placement=dict(
                    fixture=self.counter,
                    sample_region_kwargs=dict(ref=self.sink),
                    size=(0.3, 0.5),
                    pos=(1.0, -1.0),
                    try_to_place_in="container",
                ),
            )
        )

        cfgs.append(
            dict(
                name="pot",
                obj_groups=("pot"),
                mjcf_path = "robocasa/models/assets/objects/lightwheel/pot/Pot064/model.xml",
                placement=dict(
                    fixture=self.cabinet,
                    ensure_object_boundary_in_range=False,
                    size=(0.02, 0.02),
                    rotation=[(-3 * np.pi / 8, -np.pi / 4), (np.pi / 4, 3 * np.pi / 8)],
                ),
            )
        )

        cfgs.append(
            dict(
                name="egg1",
                obj_groups="egg",
                mjcf_path = None,

                graspable=True,
                placement=dict(
                    fixture=self.fridge,
                    size=(0.3, 0.3),
                    pos=(0.0, 0.0),
                    ensure_object_boundary_in_range=False,

                ),
            )
        )

        cfgs.append(
            dict(
                mjcf_path = None,

                name="egg2",
                obj_groups="egg", 
                graspable=True,
                placement=dict(
                    fixture=self.fridge,
                    size=(0.3, 0.3),
                    pos=(1.0, 0.0),
                    ensure_object_boundary_in_range=False,
                ),
            )
        )

       

        return cfgs

        
    def _reset_internal(self):
        """
        Reset the environment internal state for the stove knob tasks.
        This includes setting the stove knob state based on the behavior.
        """
        super()._reset_internal()

        '''if self.behavior == "turn_on":
            self.stove.set_knob_state(
                mode="off", knob=self.knob, env=self, rng=self.rng
            )
        elif self.behavior == "turn_off":
            self.stove.set_knob_state(mode="on", knob=self.knob, env=self, rng=self.rng)
        '''
    def _check_success(self):
        """
        Check if the microwave manipulation task is successful.

        Returns:
            bool: True if the task is successful, False otherwise.
        """
        '''turned_on = self.microwave.get_state()["turned_on"]
        gripper_button_far = self.microwave.gripper_button_far(
            self, button="start_button" if self.behavior == "turn_on" else "stop_button"
        )'''

        if self.behavior == "turn_on":
            return False
        elif self.behavior == "turn_off":
            return False
        else:
            return False



from typing import Literal
import importlib.resources as pkg_resources
import mujoco.viewer
from mujoco import MjModel, MjData, mj_name2id, mjtObj
import compliant_control.mujoco.models as models
import time
import re
from compliant_control.interface.window_commands import WindowCommands
import glfw
import numpy as np
from threading import Thread

SYNC_RATE = 60
MODEL = "arm_and_base.xml"


class Visualization:
    """Provides the mujoco visualization of the robot."""

    def __init__(self, step_cb: callable = None) -> None:
        self.step_cb = step_cb
        xml = str(pkg_resources.files(models) / MODEL)
        self.model = MjModel.from_xml_path(xml)
        self.data = MjData(self.model)
        self.define_robots()

        self.active = True

        move_target_thread = Thread(target=self.move_target_loop)
        move_target_thread.start()

    @property
    def end_effector(self) -> np.ndarray:
        """Get the position of the kinova end_effector."""
        if not self.kinova:
            return [0, 0, 0]
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        return self.data.site_xpos[idx]

    @property
    def origin_arm(self) -> np.ndarray:
        """Get the position of the origin of the kinova arm."""
        if not self.kinova:
            return [0, 0, 0]
        idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "BASE")
        return self.data.xpos[idx]

    @property
    def target(self) -> np.ndarray:
        """Get the position of the target mocap body."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target")
        return self.data.mocap_pos[body_id - 1]

    @property
    def relative_target(self) -> np.ndarray:
        """Get the target position in the arm frame."""
        return self.target - self.origin_arm

    def step(self) -> None:
        """Perform a visualization step."""
        mujoco.mj_forward(self.model, self.data)

    def set_qpos_value(
        self,
        robot: Literal["Kinova", "Dingo"],
        prop: Literal["position", "velocity"],
        values: list[float],
    ) -> None:
        """Set the joint position or velocity for kinova arm or dingo base."""
        for n, value in enumerate(values):
            idx = mj_name2id(self.model, mjtObj.mjOBJ_JOINT, f"{robot}_{n}")
            match prop:
                case "position":
                    idpos = self.model.jnt_qposadr[idx]
                    self.data.qpos[idpos] = value
                case "velocity":
                    idvel = self.model.jnt_dofadr[idx]
                    self.data.qvel[idvel]

    def start(self) -> None:
        """Start a mujoco simulation."""
        self.load_window_commands()
        viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self.key_callback
        )
        sync = time.time()
        while self.active:
            step_start = time.time()
            self.step()
            self.step_cb()
            if time.time() > sync + (1 / SYNC_RATE):
                viewer.sync()
                sync = time.time()
            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def key_callback(self, key: int) -> None:
        """Key callback."""
        if key == 256:
            self.stop()

    def stop(self, *args: any) -> None:
        """Stop the simulation."""
        self.active = False

    def update_target(self, pos: np.ndarray) -> None:
        """Update the given marker."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target")
        self.data.mocap_pos[body_id - 1] = pos

    def move_target_loop(self) -> None:
        """A loop that automatically moves the target."""
        self.automove_target = False
        frequency = 100
        target_rate = 0.05  # m/s
        target_step = target_rate / frequency
        target_rot = 90
        target_rot_rate = 30  # deg / s
        target_rot_step = target_rot_rate / frequency
        while self.active:
            if not self.automove_target:
                time.sleep(0.5)
                continue
            target_rot += target_rot_step
            step_x = np.cos(np.deg2rad(target_rot)) * target_step
            step_y = np.sin(np.deg2rad(target_rot)) * target_step
            step_z = 0
            new_target_pos = self.target + np.array([step_x, step_y, step_z])
            self.update_target(new_target_pos)
            time.sleep(1 / frequency)

    def toggle_automove_target(self) -> None:
        """Toggle automove of target."""
        print("toggle")
        self.automove_target = not self.automove_target

    def define_robots(self) -> None:
        """Define which robots are simulated."""
        names = str(self.model.names)
        self.name = re.search("b'(.*?)\\\\", names)[1]
        self.kinova = "Kinova" in names
        self.dingo = "Dingo" in names

    def load_window_commands(self) -> None:
        """Load the window commands."""
        glfw.init()
        window_commands = WindowCommands(1)
        width, height = glfw.get_video_mode(glfw.get_primary_monitor()).size
        pose = [int(width / 3), 0, int(width * (2 / 3)), height]
        window_commands = WindowCommands(1)
        window_commands.add_window(self.name)
        window_commands.add_command(["replace", (self.name, *pose)])
        window_commands.add_command(["key", (self.name, "Tab")])
        window_commands.add_command(["key", (self.name, "Shift+Tab")])
        window_commands.start_in_new_thread()
from __future__ import annotations
from typing import Literal
import os
import mujoco.viewer
from mujoco import MjModel, MjData, mj_name2id, mjtObj
import compliant_control.mujoco.models as models
import time
import re
from compliant_control.interface.window_commands import WindowCommands
import glfw
import numpy as np
from compliant_control.mujoco.target_mover import TargetMover

SYNC_RATE = 60
MODEL = "arm_and_base.xml"


def quat_rot_matrix(w: float, x: float, y: float, z: float) -> None:
    """The rotation matrix derived from a quaternion."""
    return np.array(
        [
            [
                w**2 + x**2 - y**2 - z**2,
                2 * (x * y + w * z),
                2 * (x * z - w * y),
            ],
            [
                2 * (x * y - w * z),
                w**2 - x**2 + y**2 - z**2,
                2 * (w * x + y * z),
            ],
            [
                2 * (w * y + x * z),
                2 * (y * z - w * x),
                w**2 - x**2 - y**2 + z**2,
            ],
        ]
    )


class Visualization:
    """Provides the mujoco visualization of the robot."""

    def __init__(self) -> None:
        xml = str(os.path.dirname(models.__file__) + "/" + MODEL)
        self.model = MjModel.from_xml_path(xml)
        self.data = MjData(self.model)
        self.define_robots()

        self.x0 = self.y0 = self.rotz0 = None

        self.active = True
        self.target_mover = TargetMover(self.get_target, self.update_target)

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
    def orientation_arm(self) -> np.ndarray:
        """Get the orientation of the origin of the kinova arm."""
        idx = mj_name2id(self.model, mjtObj.mjOBJ_JOINT, "D_J_B")
        idpos = self.model.jnt_qposadr[idx]
        return self.data.qpos[idpos + 3 : idpos + 7]

    @property
    def target(self) -> np.ndarray:
        """Get the position of the target mocap body."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target")
        return self.data.mocap_pos[body_id - 1]

    @property
    def relative_target(self) -> np.ndarray:
        """Get the target position in the arm frame."""
        w, x, y, z = self.orientation_arm
        pos = self.target - self.origin_arm
        return quat_rot_matrix(w, x, y, z) @ pos

    def get_target(self) -> None:
        """Return the target position."""
        return self.target

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
            if prop == "position":
                idpos = self.model.jnt_qposadr[idx]
                self.data.qpos[idpos] = value
            elif prop == "velocity":
                idvel = self.model.jnt_dofadr[idx]
                self.data.qvel[idvel]

    def set_world_pos_value(
        self, x: float, y: float, quat_w: float, quat_z: float
    ) -> None:
        """Set the world position or for the dingo base."""
        self.x0 = x if not self.x0 else self.x0
        self.y0 = y if not self.y0 else self.y0
        idx = mj_name2id(self.model, mjtObj.mjOBJ_JOINT, "D_J_B")
        idpos = self.model.jnt_qposadr[idx]
        self.data.qpos[idpos] = x - self.x0
        self.data.qpos[idpos + 1] = y - self.y0
        self.data.qpos[idpos + 3] = quat_w
        self.data.qpos[idpos + 6] = quat_z

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
        """Update the target."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target")
        self.data.mocap_pos[body_id - 1] = pos

    def reset_target(self) -> None:
        """Reset the target."""
        self.update_target(self.end_effector)

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

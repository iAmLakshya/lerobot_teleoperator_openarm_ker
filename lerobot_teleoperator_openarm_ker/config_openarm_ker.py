# Copyright 2026 Lakshya Singh Panwar. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Configuration of the OpenArm KER teleoperator."""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

ARMS = ("both", "left", "right")


@TeleoperatorConfig.register_subclass("openarm_ker")
@dataclass
class OpenArmKERConfig(TeleoperatorConfig):
    """Configuration for the OpenArm KER leader arm by Enactic.

    KER has one encoder for each joint and each gripper trigger of both arms. It streams the angles in
    degrees over USB and keeps its zero in the device.

    Each gripper trigger goes through a linear map. The trigger angle is clipped to its two
    `*_trigger_deg` values, and the released and the fully pulled trigger give the two `*_gripper_deg`
    values. The default trigger angles are the ones that Enactic's `dora-openarm-ker` node uses.

    Args:
        id (`str`, *optional*):
            Name of this KER in log messages.
        calibration_dir (`Path`, *optional*):
            Not used. KER keeps its calibration in the device.
        arms (`str`, *optional*, defaults to `"both"`):
            `"both"` gives `left_` and `right_` keys for `bi_openarm_follower`. `"left"` or `"right"` gives
            the keys of that arm with no prefix, for one `openarm_follower`.
        right_trigger_deg (`tuple[float, float]`, *optional*, defaults to `(0.0, -60.0)`):
            KER angle of the right trigger when it is released and when it is fully pulled.
        left_trigger_deg (`tuple[float, float]`, *optional*, defaults to `(0.0, 60.0)`):
            KER angle of the left trigger when it is released and when it is fully pulled.
        right_gripper_deg (`tuple[float, float]`, *optional*, defaults to `(-65.0, 0.0)`):
            Right gripper position for a released and for a fully pulled trigger. The default is open and
            closed in the gripper limits of `openarm_follower`.
        left_gripper_deg (`tuple[float, float]`, *optional*, defaults to `(-65.0, 0.0)`):
            Left gripper position for a released and for a fully pulled trigger.
        first_frame_timeout_s (`float`, *optional*, defaults to `2.0`):
            Time that `connect()` waits for the first frame after it starts the stream.
        stall_warning_s (`float`, *optional*, defaults to `0.1`):
            Time without a new frame after which `get_action()` logs a warning. It holds the last pose
            until the frames come back.
    """

    arms: str = "both"
    right_trigger_deg: tuple[float, float] = (0.0, -60.0)
    left_trigger_deg: tuple[float, float] = (0.0, 60.0)
    right_gripper_deg: tuple[float, float] = (-65.0, 0.0)
    left_gripper_deg: tuple[float, float] = (-65.0, 0.0)
    first_frame_timeout_s: float = 2.0
    stall_warning_s: float = 0.1

    def __post_init__(self) -> None:
        """Refuse an unknown value of `arms` and a trigger range of zero width."""
        if self.arms not in ARMS:
            raise ValueError(f"arms must be one of {ARMS}, not {self.arms!r}.")
        for name in ("right_trigger_deg", "left_trigger_deg"):
            released, pulled = getattr(self, name)
            if released == pulled:
                raise ValueError(f"The two angles of {name} must be different, not both {released}.")

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

"""The OpenArm KER leader arm as a LeRobot teleoperator."""

import logging
import time
from typing import Any

from lerobot.lerobot_types import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from openarm_ker import KERStream
from openarm_ker.ker_stream import CMD_STREAM

from .config_openarm_ker import OpenArmKERConfig

logger = logging.getLogger(__name__)

# The motor names of `openarm_follower`. KER channels 1 to 8 are the right arm and 9 to 16 the left arm,
# each with joints 1 to 7 and then the gripper trigger.
MOTORS = (*(f"joint_{i}" for i in range(1, 8)), "gripper")
FIRST_CHANNEL = {"right": 0, "left": 8}

_POLL_S = 0.005


class OpenArmKER(Teleoperator):
    """OpenArm KER leader arm by Enactic.

    KER is a passive leader arm for OpenArm 2.0. Its firmware applies the zero and the joint offsets, so
    the joint angles go to the follower in degrees with no change. Each gripper trigger goes through a
    linear map (see `OpenArmKERConfig`).

    If KER stops sending frames, `get_action()` holds the last pose and logs a warning. The firmware stops
    the stream when its jump detection trips, and `openarm_ker` does not restart the stream after a USB
    reconnect. In both cases the operator taps START on the KER screen to continue. The follower then
    moves to the new KER pose.
    """

    config_class = OpenArmKERConfig
    name = "openarm_ker"

    def __init__(self, config: OpenArmKERConfig):
        super().__init__(config)
        self.config = config
        self.sides = ("left", "right") if config.arms == "both" else (config.arms,)
        self._stream: KERStream | None = None
        self._frame_id: tuple[Any, Any] | None = None
        self._frame_time = 0.0
        self._stalled = False

    def _key(self, side: str, motor: str) -> str:
        prefix = f"{side}_" if self.config.arms == "both" else ""
        return f"{prefix}{motor}.pos"

    @property
    def action_features(self) -> dict[str, type]:
        """Joint and gripper positions in degrees, with the keys of the OpenArm follower."""
        return {self._key(side, motor): float for side in self.sides for motor in MOTORS}

    @property
    def feedback_features(self) -> dict[str, type]:
        """No feedback, because KER has no motors."""
        return {}

    @property
    def is_connected(self) -> bool:
        """Whether a KER session is open."""
        return self._stream is not None

    @property
    def is_calibrated(self) -> bool:
        """Always `True`, because KER keeps its calibration in the device."""
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Open KER, start its stream and wait for the first frame.

        Args:
            calibrate (`bool`, *optional*, defaults to `True`):
                Not used. KER keeps its calibration in the device.

        Raises:
            ConnectionError: If KER is not found on USB, or if an encoder of a used arm does not answer.
            TimeoutError: If no frame arrives within `first_frame_timeout_s`.
        """
        stream = KERStream()
        try:
            self._start_stream(stream)
            frame = self._wait_for_first_frame(stream)
            self._check_encoders(frame)
        except Exception:
            stream.close()
            raise

        self._stream = stream
        self._frame_id = _frame_id(frame)
        self._frame_time = time.monotonic()
        self._stalled = False
        logger.info(
            f"{self} connected: firmware {stream.metadata.get('fw')}, hardware {stream.metadata.get('hw')}."
        )

    @staticmethod
    def _start_stream(stream: KERStream) -> None:
        try:
            stream.connect()
        except (RuntimeError, OSError, ValueError) as e:
            raise ConnectionError(
                f"Could not open the OpenArm KER on USB: {e}. Connect KER directly to this computer, "
                "install libusb and, on Linux, the udev rule from the README. Only one program can use "
                "KER at a time."
            ) from e
        stream.send_command(CMD_STREAM)

    def _wait_for_first_frame(self, stream: KERStream) -> dict[str, Any]:
        deadline = time.monotonic() + self.config.first_frame_timeout_s
        while (frame := stream.latest()) is None:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"No frame from the OpenArm KER within {self.config.first_frame_timeout_s} s. "
                    "If the KER screen shows START, tap it and connect again."
                )
            time.sleep(_POLL_S)
        return frame

    def _check_encoders(self, frame: dict[str, Any]) -> None:
        errors = _error_mask(frame)
        silent = [
            f"{side} {motor}"
            for side in self.sides
            for index, motor in enumerate(MOTORS)
            if errors >> (FIRST_CHANNEL[side] + index) & 1
        ]
        if silent:
            raise ConnectionError(
                f"These KER encoders do not answer: {', '.join(silent)}. "
                "Check their cables, then restart KER."
            )

    def calibrate(self) -> None:
        """Do nothing, because KER keeps its zero in the device.

        To zero KER, lock it in its calibration jig and use "Zero Reset" on the KER screen. This class
        never sends a zero command, because that command overwrites the calibration in the device.
        """
        logger.info(f"{self} keeps its calibration in the device. There is nothing to calibrate.")

    def configure(self) -> None:
        """Do nothing. KER has no settings to apply."""

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        """Read the newest KER frame and convert it to follower positions.

        Returns:
            `RobotAction`: The position of each joint and gripper in degrees.
        """
        assert self._stream is not None  # nosec B101: the decorator checks the connection
        frame = self._stream.latest()
        self._watch_stream(frame)

        angles = frame["angles"]
        action: RobotAction = {}
        for side in self.sides:
            first = FIRST_CHANNEL[side]
            for index, motor in enumerate(MOTORS[:-1]):
                action[self._key(side, motor)] = float(angles[first + index])
            trigger_deg, gripper_deg = (
                (self.config.right_trigger_deg, self.config.right_gripper_deg)
                if side == "right"
                else (self.config.left_trigger_deg, self.config.left_gripper_deg)
            )
            action[self._key(side, "gripper")] = _trigger_to_gripper(
                angles[first + len(MOTORS) - 1], trigger_deg, gripper_deg
            )
        return action

    def _watch_stream(self, frame: dict[str, Any]) -> None:
        now = time.monotonic()
        frame_id = _frame_id(frame)
        if frame_id != self._frame_id:
            self._frame_id, self._frame_time = frame_id, now
            if self._stalled:
                self._stalled = False
                logger.info(f"{self} sends frames again. The follower moves to the KER pose.")
        elif not self._stalled and now - self._frame_time >= self.config.stall_warning_s:
            self._stalled = True
            logger.warning(
                f"{self} sent no new frame for {self.config.stall_warning_s} s. Holding the last pose. "
                "If the KER screen shows START, tap it to continue."
            )

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Not supported, because KER has no motors."""
        raise NotImplementedError("The OpenArm KER has no motors, so it takes no feedback.")

    @check_if_not_connected
    def disconnect(self) -> None:
        """Stop the KER stream and release the USB device."""
        stream, self._stream = self._stream, None
        assert stream is not None  # nosec B101: the decorator checks the connection
        stream.close()
        logger.info(f"{self} disconnected.")


def _frame_id(frame: dict[str, Any]) -> tuple[Any, Any]:
    # Firmware 2.1.0 counts frames in "seq". Firmware 2.0.0 has no counter, but its timestamp changes.
    return frame.get("seq"), frame["timestamp"]


def _error_mask(frame: dict[str, Any]) -> int:
    # Firmware 2.1.0 sends one bit for each channel. Firmware 2.0.0 sends one flag for each channel.
    if "error_mask" in frame:
        return int(frame["error_mask"])
    return sum(1 << index for index, error in enumerate(frame.get("errors", [])) if error)


def _trigger_to_gripper(
    angle: float, trigger_deg: tuple[float, float], gripper_deg: tuple[float, float]
) -> float:
    released, pulled = trigger_deg
    clipped = min(max(angle, min(released, pulled)), max(released, pulled))
    pull = (clipped - released) / (pulled - released)
    return gripper_deg[0] + pull * (gripper_deg[1] - gripper_deg[0])

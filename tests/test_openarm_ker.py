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

import importlib.metadata
import logging

import draccus
import pytest
from lerobot.robots.openarm_follower.config_openarm_follower import OpenArmFollowerConfigBase
from lerobot.teleoperators import TeleoperatorConfig, make_teleoperator_from_config
from lerobot.utils.errors import DeviceNotConnectedError
from openarm_ker.ker_stream import CMD_STREAM

from lerobot_teleoperator_openarm_ker import OpenArmKER, OpenArmKERConfig

FOLLOWER_MOTORS = list(OpenArmFollowerConfigBase(port="can0").motor_config)

# Channel n reads n degrees, so each value shows which channel it came from.
ANGLES = [float(n) for n in range(1, 17)]


def frame_v210(angles=ANGLES, error_mask=0, seq=1):
    return {"timestamp": 2000 * seq, "angles": list(angles), "error_mask": error_mask, "seq": seq}


def frame_v200(angles=ANGLES, errors=None, timestamp=2000):
    return {"timestamp": timestamp, "angles": list(angles), "errors": errors or [False] * 16}


class FakeKERStream:
    """Stands in for `openarm_ker.KERStream`. Like KER, it sends frames only after STREAM."""

    def __init__(self, frame):
        self.frame = frame
        self.commands = []
        self.closed = False
        self.metadata = {"fw": "2.1.0", "hw": "2.0.0"}

    def connect(self):
        pass

    def send_command(self, command):
        self.commands.append(command)

    def latest(self):
        return self.frame if CMD_STREAM in self.commands else None

    def close(self):
        self.closed = True


@pytest.fixture
def ker(monkeypatch):
    fake = FakeKERStream(frame_v210())
    monkeypatch.setattr("lerobot_teleoperator_openarm_ker.openarm_ker.KERStream", lambda: fake)
    return fake


def make_teleop(tmp_path, **kwargs):
    return OpenArmKER(OpenArmKERConfig(calibration_dir=tmp_path, **kwargs))


def test_action_keys_match_bi_openarm_follower(tmp_path):
    teleop = make_teleop(tmp_path)

    expected = {f"{side}_{motor}.pos" for side in ("left", "right") for motor in FOLLOWER_MOTORS}
    assert set(teleop.action_features) == expected
    assert teleop.feedback_features == {}


def test_channels_map_to_joints_in_degrees(tmp_path, ker):
    teleop = make_teleop(tmp_path)
    teleop.connect()

    action = teleop.get_action()

    assert set(action) == set(teleop.action_features)
    assert [action[f"right_joint_{i}.pos"] for i in range(1, 8)] == ANGLES[0:7]
    assert [action[f"left_joint_{i}.pos"] for i in range(1, 8)] == ANGLES[8:15]


def test_one_arm_gives_openarm_follower_keys(tmp_path, ker):
    teleop = make_teleop(tmp_path, arms="left")
    teleop.connect()

    action = teleop.get_action()

    assert set(action) == {f"{motor}.pos" for motor in FOLLOWER_MOTORS}
    assert [action[f"joint_{i}.pos"] for i in range(1, 8)] == ANGLES[8:15]


@pytest.mark.parametrize(
    ("side", "channel", "trigger_deg", "gripper_deg"),
    [
        ("right", 7, 0.0, -65.0),  # released: open
        ("right", 7, -60.0, 0.0),  # fully pulled: closed
        ("right", 7, -30.0, -32.5),
        ("right", 7, -90.0, 0.0),  # past the pull: clipped
        ("right", 7, 20.0, -65.0),  # past the release: clipped
        ("left", 15, 0.0, -65.0),
        ("left", 15, 60.0, 0.0),
        ("left", 15, 30.0, -32.5),
        ("left", 15, 90.0, 0.0),
        ("left", 15, -20.0, -65.0),
    ],
)
def test_trigger_maps_to_follower_gripper(tmp_path, ker, side, channel, trigger_deg, gripper_deg):
    angles = list(ANGLES)
    angles[channel] = trigger_deg
    ker.frame = frame_v210(angles)
    teleop = make_teleop(tmp_path)
    teleop.connect()

    assert teleop.get_action()[f"{side}_gripper.pos"] == pytest.approx(gripper_deg)


def test_one_gripper_option_from_the_command_line_keeps_the_other_defaults():
    # lerobot-teleoperate parses the --teleop.* options with draccus.
    config = draccus.parse(OpenArmKERConfig, args=["--left_gripper_deg=[65, 0]"])

    assert config.left_gripper_deg == (65.0, 0.0)
    assert config.left_trigger_deg == (0.0, 60.0)
    assert config.right_trigger_deg == (0.0, -60.0)
    assert config.right_gripper_deg == (-65.0, 0.0)


def test_connect_fails_and_releases_ker_when_no_frame_arrives(tmp_path, ker):
    ker.frame = None
    teleop = make_teleop(tmp_path, first_frame_timeout_s=0.05)

    with pytest.raises(TimeoutError, match="START"):
        teleop.connect()
    assert ker.closed
    assert not teleop.is_connected


@pytest.mark.parametrize(
    "frame",
    [frame_v210(error_mask=1 << 10), frame_v200(errors=[channel == 10 for channel in range(16)])],
    ids=["firmware-2.1.0", "firmware-2.0.0"],
)
def test_connect_refuses_an_encoder_that_does_not_answer(tmp_path, ker, frame):
    ker.frame = frame
    teleop = make_teleop(tmp_path)

    with pytest.raises(ConnectionError, match="left joint_3"):
        teleop.connect()
    assert ker.closed
    assert not teleop.is_connected


def test_encoder_error_on_the_unused_arm_does_not_block(tmp_path, ker):
    ker.frame = frame_v210(error_mask=1 << 10)
    teleop = make_teleop(tmp_path, arms="right")

    teleop.connect()

    assert teleop.is_connected


@pytest.mark.parametrize(
    ("first", "after"),
    [
        (frame_v210(seq=1), frame_v210([a + 1 for a in ANGLES], seq=2)),
        (frame_v200(timestamp=2000), frame_v200([a + 1 for a in ANGLES], timestamp=4000)),
    ],
    ids=["firmware-2.1.0", "firmware-2.0.0"],
)
def test_holds_last_pose_when_frames_stop_and_follows_when_they_return(tmp_path, ker, caplog, first, after):
    ker.frame = first
    teleop = make_teleop(tmp_path, stall_warning_s=0.0)
    teleop.connect()

    with caplog.at_level(logging.WARNING):
        held = teleop.get_action()
    assert "Holding the last pose" in caplog.text
    assert held["right_joint_1.pos"] == 1.0

    ker.frame = after
    assert teleop.get_action()["right_joint_1.pos"] == 2.0


def test_calibrate_sends_nothing_to_ker(tmp_path, ker):
    teleop = make_teleop(tmp_path)
    teleop.connect()

    teleop.calibrate()

    assert ker.commands == [CMD_STREAM]
    assert teleop.is_calibrated


def test_disconnect_releases_ker(tmp_path, ker):
    teleop = make_teleop(tmp_path)
    teleop.connect()

    teleop.disconnect()

    assert ker.closed
    assert not teleop.is_connected
    with pytest.raises(DeviceNotConnectedError):
        teleop.get_action()


def test_lerobot_finds_the_plugin_by_its_package_name(tmp_path):
    # LeRobot imports every installed distribution whose name starts with "lerobot_teleoperator_".
    names = {dist.metadata["Name"] for dist in importlib.metadata.distributions()}
    assert "lerobot_teleoperator_openarm_ker" in names

    config = TeleoperatorConfig.get_choice_class("openarm_ker")(calibration_dir=tmp_path)
    assert isinstance(make_teleoperator_from_config(config), OpenArmKER)

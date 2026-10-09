# lerobot_teleoperator_openarm_ker

[![CI](https://github.com/iAmLakshya/lerobot_teleoperator_openarm_ker/actions/workflows/ci.yml/badge.svg)](https://github.com/iAmLakshya/lerobot_teleoperator_openarm_ker/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/lerobot_teleoperator_openarm_ker)](https://pypi.org/project/lerobot_teleoperator_openarm_ker/)

A [LeRobot](https://github.com/huggingface/lerobot) teleoperator for the [OpenArm KER](https://docs.openarm.dev/hardware/openarm-ker/general) leader arm by Enactic. KER drives the LeRobot `bi_openarm_follower`, or one `openarm_follower`.

> **Status:** the unit tests pass with LeRobot 0.6.1. This version is not yet tested with a real KER and real OpenArm arms.

## Install

```bash
pip install lerobot_teleoperator_openarm_ker
```

LeRobot finds the plugin by its package name. You do not have to import it.

KER uses a USB vendor class, not a serial port. Its library, [`openarm_ker`](https://github.com/enactic/openarm_ker), needs libusb:

- **Ubuntu:** install `libusb-1.0-0`, then give your user access to KER:

  ```bash
  echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4002", MODE="0666"' \
    | sudo tee /etc/udev/rules.d/99-openarm-ker.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

- **macOS:** `brew install libusb`.
- **Windows:** not tested.

Connect KER directly to a USB port of the computer. Behind a hub or a dock, KER can stay in its boot mode ([firmware issue 7](https://github.com/enactic/openarm_ker_firmware/issues/7)). Only one program can use KER at a time.

## Use

Read [Before the first session](#before-the-first-session) first.

Both arms:

```bash
lerobot-teleoperate \
  --robot.type=bi_openarm_follower \
  --robot.id=my_openarm \
  --robot.left_arm_config.port=can0 \
  --robot.left_arm_config.side=left \
  --robot.left_arm_config.max_relative_target=5 \
  --robot.right_arm_config.port=can1 \
  --robot.right_arm_config.side=right \
  --robot.right_arm_config.max_relative_target=5 \
  --teleop.type=openarm_ker
```

One arm:

```bash
lerobot-teleoperate \
  --robot.type=openarm_follower \
  --robot.id=my_follower \
  --robot.port=can0 \
  --robot.side=right \
  --robot.max_relative_target=5 \
  --teleop.type=openarm_ker \
  --teleop.arms=right
```

`lerobot-record` takes the same `--robot` and `--teleop` arguments.

## Behaviour

- **Joints.** The KER firmware applies the zero and the joint offsets. The plugin sends each joint angle to the follower in degrees with no change.
- **Grippers.** A linear map converts each trigger to a gripper position. A released trigger gives -65° (open), and a fully pulled trigger gives 0° (closed). These are the gripper limits of `openarm_follower`. The trigger ranges (0° to -60° on the right, 0° to 60° on the left) come from Enactic's [`dora-openarm-ker`](https://github.com/enactic/dora-openarm-ker). To change the map, give the released and the pulled value, for example `--teleop.left_gripper_deg="[65, 0]"`. The options are `right_trigger_deg`, `left_trigger_deg`, `right_gripper_deg` and `left_gripper_deg`.
- **Calibration.** None. KER keeps its zero in the device. The plugin never sends a zero command, because that command overwrites the calibration in the device. To zero KER, follow the [KER calibration workflow](https://docs.openarm.dev/hardware/openarm-ker/calibration-workflow).
- **Encoders.** `connect()` stops with an error if an encoder of a used arm did not answer when KER started.
- **Stream stops.** KER stops its stream in two cases. The first is its jump detection: a joint angle stays 30° or more away from its last good value for 20 frames. The second is a reconnect of the USB link. The plugin then holds the last pose and logs a warning. To continue, tap START on the KER screen. The follower then moves to the KER pose at once, unless `max_relative_target` limits the step.

## Before the first session

1. **Check the follower zero.** KER and the follower must use the same zero: the arm hangs straight down and the gripper is closed. `lerobot-calibrate` asks for this pose. If no calibration file exists for `--robot.id`, `openarm_follower` sets the motor zeros again when it connects.
2. **Check the mapping with no motion.** Run `python -m lerobot_teleoperator_openarm_ker` and hold KER in the zero pose. Every joint must read about 0°. Then compare each joint and each gripper with the follower in the same pose, with the follower torque off. Check the direction of the left gripper in particular.
3. **Set `max_relative_target`.** It limits each step of the follower. At the default 60 fps of `lerobot-teleoperate`, a value of 5 limits each joint to 300° per second.

## Compatibility

| Component | Version |
|---|---|
| LeRobot | 0.6.1 |
| `openarm_ker` | 0.3.0 |
| KER firmware | 2.0.0 and 2.1.0 frame formats, in the unit tests only |
| Python | 3.12 |

## Development

```bash
uv sync
uv run pytest
uv run ruff check . && uv run ruff format --check .
uv run mypy
```

## Licence

Apache-2.0. See [LICENSE](https://github.com/iAmLakshya/lerobot_teleoperator_openarm_ker/blob/main/LICENSE).

KER, `openarm_ker` and the KER firmware are by Enactic, Inc., under Apache-2.0. This project is not affiliated with Enactic or Hugging Face.

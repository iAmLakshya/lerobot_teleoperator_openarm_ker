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

"""Print the actions of the OpenArm KER with no robot. Nothing moves.

Use it before the first teleoperation: put the follower in the same pose with its torque off, and
compare its joint positions with these values.

    python -m lerobot_teleoperator_openarm_ker --arms both
"""

import argparse
import time

from .config_openarm_ker import ARMS, OpenArmKERConfig
from .openarm_ker import MOTORS, OpenArmKER


def main() -> None:
    """Connect to KER and print its actions until Ctrl+C."""
    parser = argparse.ArgumentParser(description="Print the OpenArm KER actions. Nothing moves.")
    parser.add_argument("--arms", choices=ARMS, default="both")
    parser.add_argument("--hz", type=float, default=5.0)
    args = parser.parse_args()

    header = "      " + "".join(f"{motor:>9}" for motor in MOTORS)
    with OpenArmKER(OpenArmKERConfig(arms=args.arms)) as ker:
        try:
            while True:
                action = ker.get_action()
                print(header)
                for side in ker.sides:
                    prefix = f"{side}_" if args.arms == "both" else ""
                    print(f"{side:>5} " + "".join(f"{action[f'{prefix}{m}.pos']:9.1f}" for m in MOTORS))
                time.sleep(1 / args.hz)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()

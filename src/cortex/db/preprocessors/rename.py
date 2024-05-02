#  Copyright (c) 2024 Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#
#      https://www.apache.org/licenses/LICENSE-2.0
#
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import typing


def rename(mappings: dict, data: typing.List[dict]):
    """Rename keys in a list of dictionaries.

    Args:
        mappings: A dictionary of old_key: new_key pairs.
        data: A list of dictionaries (rows) to modify.
    """
    for row in data:
        # Modify the data in place with updated keys
        for old_key, new_key in mappings.items():
            if old_key in row:
                row[new_key] = row.pop(old_key)
    return data


def test():
    rows = [
        dict(name="joint_bend_01", pos=-0.03, velocity=0.0, effort=0.0),
        dict(name="joint_bend_02", pos=0.532, velocity=0.0, effort=0.0),
        dict(velocity=1.0),
        dict(
            actuator="joint_bend_02", pos=0.532, velocity=0.0, effort=0.0, torque=0.43
        ),
    ]
    mappings = {"name": "actuator", "pos": "position"}
    result = rename(mappings, rows)
    assert result == [
        dict(actuator="joint_bend_01", position=-0.03, velocity=0.0, effort=0.0),
        dict(actuator="joint_bend_02", position=0.532, velocity=0.0, effort=0.0),
        dict(velocity=1.0),
        dict(
            actuator="joint_bend_02",
            position=0.532,
            velocity=0.0,
            effort=0.0,
            torque=0.43,
        ),
    ]

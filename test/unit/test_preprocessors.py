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

import unittest
from cortex.db.preprocessors import flatten, rename, parse_timestamps


class TestPreprocessors(unittest.TestCase):
    def test_flatten(self):
        data = dict(a=[1, 2, 3, 4], b=[4, 5, 6], c=[7, 8, 9, 10])
        fields = ["a", "b"]
        expected = [dict(a=1, b=4), dict(a=2, b=5), dict(a=3, b=6)]
        result = flatten(fields, [data])
        self.assertEqual(result, expected)

    def test_rename(self):
        mappings = {"name": "actuator", "pos": "position"}

        rows = [
            dict(name="joint_bend_01", pos=-0.03, velocity=0.0, effort=0.0),
            dict(name="joint_bend_02", pos=0.532, velocity=0.0, effort=0.0),
            dict(velocity=1.0),
            dict(
                actuator="joint_bend_02",
                pos=0.532,
                velocity=0.0,
                effort=0.0,
                torque=0.43,
            ),
        ]
        expected = [
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

        result = rename(mappings, rows)
        self.assertEqual(result, expected)

    def test_parse_timestamps(self):
        fields = ["CreatedAt", "UpdatedAt", "ros_time"]
        data = [
            dict(
                CreatedAt="01/01/2021 00:00:00",
                UpdatedAt="2021-01-01T00:00:00Z",
                ros_time=1612147200.0,
            )
        ]
        expected = [
            dict(CreatedAt=1612147200.0, UpdatedAt=1612147200.0, ros_time=1612147200.0)
        ]
        result = parse_timestamps(fields, data)

        print(f"Result: ", result)
        print(f"Expected: ", expected)

        self.assertEqual(result, expected)


if __name__ == "__main__":
    unittest.main()

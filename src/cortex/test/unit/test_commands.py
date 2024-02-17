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

import time
import unittest
from cortex.agents import CRTXCommander, CRTXObserver


class TestCommands(unittest.TestCase):
    commander = CRTXCommander()
    observer = CRTXObserver()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_move_forward(self):
        """Test the robot's ability to move forward."""
        time.sleep(1)
        self.assertTrue(True, "This is a placeholder")

    def test_move_backward(self):
        """Test the robot's ability to move backward."""
        time.sleep(1)
        self.assertTrue(True, "This is a placeholder")

    def test_stop(self):
        """Test the robot's ability to stop."""
        time.sleep(1)
        self.assertTrue(True, "This is a placeholder")

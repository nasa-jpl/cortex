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

from cortex.types import CRTXTestRunner
from cortex.test.integration import CRTXIntegrationTest
from cortex.test.unit import TestCommands


class TestROS1(CRTXIntegrationTest):
    def __init__(self):
        super(TestROS1, self).__init__()
        self.addTest(TestCommands('test_move_forward'))
        self.addTest(TestCommands('test_stop'))
        self.addTest(TestCommands('test_move_backward'))
        self.addTest(TestCommands('test_stop'))


if __name__ == '__main__':
    result = CRTXTestRunner().run(TestROS1())
    print(result)

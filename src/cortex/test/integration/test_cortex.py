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
from cortex.test.internal import TestTemporalCRTX, TestEnvironment, TestBasicWorker, TestMonitor


class TestCORTEX(CRTXIntegrationTest):
    test_name = "CORTEX Integration Test"

    def __init__(self):
        super(TestCORTEX, self).__init__()

        # Environment Tests
        self.addTest(TestEnvironment('test_local_defaults'))
        self.addTest(TestEnvironment('test_local_modifications'))
        self.addTest(TestEnvironment('test_key_values'))

        # Database Tests
        self.addTest(TestTemporalCRTX('test_connection'))
        self.addTest(TestTemporalCRTX('test_insertion'))

        # Worker Tests
        self.addTest(TestBasicWorker('test_basic_config'))
        self.addTest(TestBasicWorker('test_basic_worker_sample_rate'))

        # Monitor Tests
        self.addTest(TestMonitor('test_nominal_monitor'))
        self.addTest(TestMonitor('test_live_insertion'))
        self.addTest(TestMonitor('test_duplicate_node'))
        self.addTest(TestMonitor('test_throughput'))
        self.addTest(TestMonitor('test_killed_process'))


if __name__ == '__main__':
    result = CRTXTestRunner().run(TestCORTEX())
    print(result)

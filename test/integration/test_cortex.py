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

from cortex.types import CRTXTestRunner, CRTXIntegrationTest
from test.unit import (
    TestCRTXDatabase,
    TestCRTXEnvironment,
    TestBasicCRTXWorker,
    TestCRTXMonitor,
)


class TestCORTEX(CRTXIntegrationTest):
    test_name = "CORTEX Integration Test"

    def __init__(self):
        super(TestCORTEX, self).__init__()

        # Environment Tests
        self.addTest(TestCRTXEnvironment("test_local_defaults"))
        self.addTest(TestCRTXEnvironment("test_local_modifications"))
        self.addTest(TestCRTXEnvironment("test_key_values"))

        # Database Tests
        self.addTest(TestCRTXDatabase("test_connection"))
        self.addTest(TestCRTXDatabase("test_insertion"))

        # Worker Tests
        self.addTest(TestBasicCRTXWorker("test_basic_config"))
        self.addTest(TestBasicCRTXWorker("test_basic_worker_sample_rate"))

        # Monitor Tests
        self.addTest(TestCRTXMonitor("test_nominal_monitor"))
        self.addTest(TestCRTXMonitor("test_live_insertion"))
        self.addTest(TestCRTXMonitor("test_duplicate_node"))
        self.addTest(TestCRTXMonitor("test_throughput"))
        self.addTest(TestCRTXMonitor("test_killed_process"))


if __name__ == "__main__":
    result = CRTXTestRunner().run(TestCORTEX())
    print(result)

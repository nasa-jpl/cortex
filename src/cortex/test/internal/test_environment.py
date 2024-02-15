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

import os
import datetime
import unittest
from cortex.config import CRTXEnvironment


class TestEnvironment(unittest.TestCase):
    def setUp(self):
        # Copy the current environment variables
        self.env = os.environ.copy()

    # Specify the order in which to run the tests
    def tearDown(self):
        # Reset the environment variables
        os.environ.clear()
        os.environ.update(self.env)

    def test_local_defaults(self):
        """Make sure the local environment is set to the system's hostname and default values are set correctly."""
        local_env = CRTXEnvironment.local()
        self.assertEqual(local_env.device.HOSTNAME, os.uname().nodename, "Default hostname should be the system's hostname")
        self.assertEqual(local_env.system.DB_HOSTNAME, '127.0.0.1', "Default DB hostname should be 127.0.0.1")
        self.assertEqual(local_env.system.DB_PORT, '5432', "Default DB port should be 5432")
        self.assertEqual(local_env.system.MONITOR_HZ, '1.0', "Default monitor hz should be 1.0")
        self.assertEqual(local_env.system.RUN_NUMBER, '0', "Default run number should be 0")
        self.assertEqual(local_env.system.RUN_LABEL, 'default', "Default run label should be 'default'")
        self.assertEqual(local_env.system.RUN_DESCRIPTION, '', "Default run description should be empty")

        # Need to parse ISO 8601 date string
        run_start_time = datetime.datetime.fromisoformat(local_env.system.RUN_START_TIME)
        # Assert the start time was set within the last 5 seconds
        self.assertLess((datetime.datetime.now() - run_start_time).total_seconds(), 5, "Start time was not set within the last 5 seconds")

    def test_key_values(self):
        """Test that the key-value pairs are set correctly."""
        env = CRTXEnvironment.local()
        for key, value in env:
            self.assertEqual(getattr(env, key), value, f"Key {key} should be {value}, not {getattr(env, key)}")

        system = env.system
        for key, value in system:
            self.assertEqual(getattr(system, key), value, f"Key {key} should be {value}, not {getattr(system, key)}")

        device = env.device
        for key, value in device:
            self.assertEqual(getattr(device, key), value, f"Key {key} should be {value}, not {getattr(device, key)}")

    def test_local_modifications(self):
        """Modify the environment variables and test that the local environment is properly adapted."""
        # Set the environment variables to test values
        os.environ['HOSTNAME'] = 'test-hostname'
        os.environ['DB_HOSTNAME'] = 'test-db-hostname'
        os.environ['DB_PORT'] = 'test-db-port'
        os.environ['RUN_NUMBER'] = 'test-run-number'
        os.environ['RUN_START_TIME'] = 'test-run-start-time'
        os.environ['RUN_LABEL'] = 'test-run-label'
        os.environ['RUN_DESCRIPTION'] = 'test-run-description'

        local_env = CRTXEnvironment.local()

        # Assert that the environment variables were set correctly
        self.assertEqual(local_env.system.DB_HOSTNAME, 'test-db-hostname', f"DB_HOSTNAME should be 'test-db-hostname', not {local_env.system.DB_HOSTNAME}")
        self.assertEqual(local_env.system.DB_PORT, 'test-db-port', f"DB_PORT should be 'test-db-port', not {local_env.system.DB_PORT}")
        self.assertEqual(local_env.system.RUN_NUMBER, 'test-run-number', f"RUN_NUMBER should be 'test-run-number', not {local_env.system.RUN_NUMBER}")
        self.assertEqual(local_env.system.RUN_START_TIME, 'test-run-start-time', f"RUN_START_TIME should be 'test-run-start-time', not {local_env.system.RUN_START_TIME}")
        self.assertEqual(local_env.system.RUN_LABEL, 'test-run-label', f"RUN_LABEL should be 'test-run-label', not {local_env.system.RUN_LABEL}")
        self.assertEqual(local_env.system.RUN_DESCRIPTION, 'test-run-description', f"RUN_DESCRIPTION should be 'test-run-description', not {local_env.system.RUN_DESCRIPTION}")
        self.assertEqual(local_env.device.HOSTNAME, 'test-hostname', f"Device hostname should be 'test-hostname', not {local_env.device.HOSTNAME}")

        os.environ.clear()
        os.environ.update(self.env)

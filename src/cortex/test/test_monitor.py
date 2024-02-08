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

import multiprocessing
import time
import unittest
from cortex.agents import Monitor
from cortex.db import TemporalCRTX
from cortex.db.entities import NodeResourceUtilization


class TestMonitor(unittest.TestCase):
    def setUp(self):
        self.DB_HOSTNAME = '127.0.0.1'
        self.DB_PORT = '5432'
        self.db = TemporalCRTX(self.DB_HOSTNAME, self.DB_PORT, logging=False)
        self.robot_name = 'Test'

    def tearDown(self):
        # Delete all entries that use the test robot name
        with self.db.get_session() as session:
            stats = session.query(NodeResourceUtilization).filter(
                NodeResourceUtilization.robot == self.robot_name).all()
            for stat in stats:
                session.delete(stat)
            session.commit()
        time.sleep(3)
        self.db.shutdown(block=True)

    def test_nominal_monitor(self):
        """Verify that the monitor can collect stats from a running process."""
        MONITOR_HZ = 5
        SLEEP_TIME = 5

        # Open two long-running processes with the same name
        p1 = multiprocessing.Process(target=lambda: time.sleep(1000))
        p1.start()

        # Assumes that the database is running locally. If not, change the IP address and port number accordingly
        m = Monitor(self.DB_HOSTNAME, self.DB_PORT, hz=MONITOR_HZ)
        m.add_node(p1.pid, f"python_test_proc_{p1.pid}")
        m.start()

        # Wait to ensure the stats are collected
        time.sleep(SLEEP_TIME)
        m.stop()
        p1.terminate()

        # Retrieve stats from the database in the last 5 seconds
        with self.db.get_session() as session:
            stats = session.query(NodeResourceUtilization).filter(
                NodeResourceUtilization.node == f"python_test_proc_{p1.pid}").all()

            # We should have approximately MONITOR_HZ * SLEEP_TIME stats
            assert len(stats) > (MONITOR_HZ * SLEEP_TIME) - (
                        MONITOR_HZ * SLEEP_TIME * 0.1), "Monitor did not collect enough stats."

            for stat in stats:
                assert stat.cpu_percent >= 0.0, "Expected non-negative CPU usage"
                assert stat.memory_percent > 0.0, "Expected non-zero memory usage"
                assert stat.num_threads >= 1, "Expected at least one thread"
                assert stat.num_fds >= 1, "Expected at least one file descriptor"
                assert stat.status in ["running", "sleeping"], f"Unexpected process status: {stat.status}"

                # Delete the stats from the database
                session.delete(stat)
            session.commit()

        time.sleep(3)

        # Remove node from the monitor
        m.remove_node(f"python_test_proc_{p1.pid}")

        # Assert that no stats are collected
        stats = m.get_stats()
        assert len(stats) == 0, "Nodes were not removed from monitor"

    def test_live_insertion(self):
        """Verify that the monitor can collect stats from a running process that is inserted after the monitor starts."""
        # Open two long-running processes with the same name
        p1 = multiprocessing.Process(target=lambda: time.sleep(2000))
        p2 = multiprocessing.Process(target=lambda: time.sleep(500))
        p1.start()
        p2.start()

        # Get the PIDs of the processes
        m = Monitor(self.DB_HOSTNAME, self.DB_PORT)
        m.add_node(p1.pid, f"python_test_proc_{p1.pid}")
        m.start()

        # Wait a few seconds and add the second process
        time.sleep(3)
        m.add_node(p2.pid, f"python_test_proc_{p2.pid}")

        # Wait a few more seconds to ensure the stats are collected
        time.sleep(3)
        stats = m.get_stats()

        # Kill the processes
        p1.terminate()
        p2.terminate()
        m.stop()

        assert len(stats) == 2, "Expected stats for both nodes"

    def test_duplicate_node(self):
        """Verify that the monitor can handle duplicate nodes."""
        # Open two long-running processes with the same name
        p1 = multiprocessing.Process(target=lambda: time.sleep(2000))
        p2 = multiprocessing.Process(target=lambda: time.sleep(500))
        p1.start()
        p2.start()

        # Get the PIDs of the processes
        m = Monitor(self.DB_HOSTNAME, self.DB_PORT)

        try:
            m.add_node(p1.pid, f"python_test_proc_{p1.pid}")
            m.add_node(p2.pid, f"python_test_proc_{p1.pid}")
            assert False, "Expected ValueError for duplicate node"
        except ValueError:
            assert True, "Expected ValueError for duplicate node"

        p1.terminate()
        p2.terminate()

    def test_throughput(self):
        """Verify that the monitor can successfully collect stats at a high rate from multiple processes."""
        NUM_PROCESSES = 100
        MONITOR_HZ = 1
        SLEEP_TIME = 5
        procs = []

        for i in range(NUM_PROCESSES):
            p = multiprocessing.Process(target=lambda: time.sleep(1000))
            procs.append(p)

        m = Monitor(self.DB_HOSTNAME, self.DB_PORT, hz=MONITOR_HZ)
        for p in procs:
            p.start()
            m.add_node(p.pid, f"python_test_proc_{p.pid}")
        m.start()

        # Wait to ensure the stats are collected (plus a little extra)
        time.sleep(SLEEP_TIME + (SLEEP_TIME * 0.15))
        m.stop(delay=SLEEP_TIME)
        for p in procs:
            p.terminate()
        # Give the database time to insert all entries
        time.sleep(SLEEP_TIME)

        # Retrieve stats from the database in the last 5 seconds
        with self.db.get_session() as session:
            stats = session.query(NodeResourceUtilization).filter(
                NodeResourceUtilization.robot == self.robot_name).all()

            # There should be 100 unique process names
            names = set([stat.node for stat in stats])
            assert len(names) == NUM_PROCESSES, "Expected stats for all processes"

            # We should have approximately MONITOR_HZ * SLEEP_TIME * NUM_PROCESSES stats
            assert len(stats) > (MONITOR_HZ * SLEEP_TIME * NUM_PROCESSES) - (
                    MONITOR_HZ * SLEEP_TIME * NUM_PROCESSES * 0.1), "Monitor did not collect enough stats."

            for stat in stats:
                assert stat.cpu_percent >= 0.0, "Expected non-negative CPU usage"
                assert stat.memory_percent > 0.0, "Expected non-zero memory usage"
                assert stat.num_threads >= 1, "Expected at least one thread"
                assert stat.num_fds >= 1, "Expected at least one file descriptor"
                assert stat.status in ["running", "sleeping"], f"Unexpected process status: {stat.status}"

                # Delete the stats from the database
                session.delete(stat)
            session.commit()

    def test_killed_process(self):
        """Verify that the monitor properly handles terminated nodes."""
        SLEEP_TIME = 3

        # Open two long-running processes with the same name
        p1 = multiprocessing.Process(target=lambda: time.sleep(1000))
        p1.start()

        # Get the PIDs of the processes
        m = Monitor(self.DB_HOSTNAME, self.DB_PORT)
        m.add_node(p1.pid, f"python_test_proc_{p1.pid}")
        m.start()

        # Wait for a bit and ensure the stats are collected
        time.sleep(SLEEP_TIME)
        stats = m.get_stats()
        assert len(stats) == 1, "Expected stats for one node"

        # Kill the node and ensure the monitor removes it automatically
        p1.terminate()
        time.sleep(SLEEP_TIME)
        m.stop()
        stats = m.get_stats()
        assert len(stats) == 0, "Expected monitor to remove node after termination"

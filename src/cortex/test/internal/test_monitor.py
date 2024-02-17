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
import contextlib
import math
import multiprocessing
import time
import unittest
from cortex.agents import CRTXMonitor
from cortex.db import TemporalCRTX
from cortex.db.entities import NodeResourceUtilization


class TestCRTXMonitor(unittest.TestCase):
    def setUp(self):
        self.DB_HOSTNAME = "127.0.0.1"
        self.DB_PORT = "5432"
        self.robot_name = "Test Robot"
        self.db = TemporalCRTX(
            self.DB_HOSTNAME, self.DB_PORT, batch_timeout=1, logging=False
        )

    def tearDown(self):
        # Delete all entries that use the test robot name
        self.db.delete(
            NodeResourceUtilization, NodeResourceUtilization.robot == self.robot_name
        )
        self.db.delete(
            NodeResourceUtilization,
            NodeResourceUtilization.node.like(f"python_test_proc%"),
        )
        self.db.shutdown(block=True)

    def create_processes(self, count=1, start=True):
        procs = []
        for i in range(count):
            p = multiprocessing.Process(target=lambda: time.sleep(1000))
            if start:
                p.start()
            procs.append(p)
        return procs

    def kill_processes(self, procs):
        for p in procs:
            p.terminate()

    @contextlib.contextmanager
    def processes(self, count=1):
        procs = self.create_processes(count)
        yield procs
        self.kill_processes(procs)

    def test_nominal_monitor(self):
        """Verify that the monitor can collect stats from a running process."""
        monitor_hz = sleep_time = 5
        p1 = self.create_processes(count=1, start=True)[0]

        # Assumes that the database is running locally. If not, change the IP address and port number accordingly
        with CRTXMonitor.at_rate(monitor_hz) as m:
            m.add_node(p1.pid, f"python_test_proc_{p1.pid}")
            m.start()
            time.sleep(sleep_time)
        self.kill_processes([p1])

        # Retrieve stats from the database in the last 5 seconds
        stats = self.db.query(
            NodeResourceUtilization,
            NodeResourceUtilization.node == f"python_test_proc_{p1.pid}",
        )

        # We should have approximately MONITOR_HZ * SLEEP_TIME stats
        expected = monitor_hz * sleep_time
        expected = math.floor(expected)
        self.assertGreaterEqual(
            len(stats), expected, "Monitor did not collect enough stats."
        )

        # Verify that the stats are valid
        for stat in stats:
            self.assertGreaterEqual(
                stat.cpu_percent, 0.0, "Expected non-negative CPU usage"
            )
            self.assertGreater(
                stat.memory_percent, 0.0, "Expected non-zero memory usage"
            )
            self.assertGreaterEqual(stat.num_threads, 1, "Expected at least one thread")
            self.assertGreaterEqual(
                stat.num_fds, 1, "Expected at least one file descriptor"
            )
            self.assertIn(
                stat.status,
                ["running", "sleeping"],
                f"Unexpected process status: {stat.status}",
            )

    def test_live_insertion(self):
        """Verify that the monitor can collect stats from a running process that is inserted after the monitor starts."""
        with CRTXMonitor.at_rate(0.5) as m:
            with self.processes(2) as procs:
                m.add_node(procs[0].pid, f"python_test_proc_{procs[0].pid}")
                m.start()

                # Wait a few seconds and add the second process
                time.sleep(3)
                m.add_node(procs[1].pid, f"python_test_proc_{procs[1].pid}")

                # Wait a few more seconds to ensure the stats are collected
                time.sleep(1)
                stats = m.get_stats()
        self.assertEqual(len(stats), 2, "Expected stats for both nodes")

    def test_duplicate_node(self):
        """Verify that the monitor correctly rejects duplicate nodes."""
        with CRTXMonitor.at_rate(1) as m:
            with self.processes(2) as procs:

                def add_duplicate_node():
                    m.add_node(procs[0].pid, f"python_test_proc_{procs[0].pid}")
                    m.add_node(procs[1].pid, f"python_test_proc_{procs[0].pid}")

                self.assertRaises(ValueError, add_duplicate_node)

    def test_throughput(self):
        """Verify that the monitor can successfully collect stats at a high rate from multiple processes."""
        n_procs = 100
        monitor_hz = 1
        sleep_time = 5

        with CRTXMonitor.at_rate(monitor_hz) as m:
            with self.processes(n_procs) as procs:
                for p in procs:
                    m.add_node(p.pid, f"python_test_proc_{p.pid}")
                m.start()
                time.sleep(sleep_time)

        stats = self.db.query(
            NodeResourceUtilization, NodeResourceUtilization.robot == self.robot_name
        )
        names = set([stat.node for stat in stats])
        self.assertEqual(len(names), n_procs, "Expected stats for all processes")

        expected = monitor_hz * sleep_time * n_procs
        self.assertGreaterEqual(
            len(stats), expected, "Monitor did not collect enough stats."
        )

        for stat in stats:
            self.assertGreaterEqual(
                stat.cpu_percent, 0.0, "Expected non-negative CPU usage"
            )
            self.assertGreater(
                stat.memory_percent, 0.0, "Expected non-zero memory usage"
            )
            self.assertGreaterEqual(stat.num_threads, 1, "Expected at least one thread")
            self.assertGreaterEqual(
                stat.num_fds, 1, "Expected at least one file descriptor"
            )
            self.assertIn(
                stat.status,
                ["running", "sleeping"],
                f"Unexpected process status: {stat.status}",
            )

    def test_killed_process(self):
        """Verify that the monitor properly handles terminated nodes."""
        sleep_time = 2
        with CRTXMonitor.at_rate(1) as m:
            with self.processes(1) as procs:
                m.add_node(procs[0].pid, f"python_test_proc_{procs[0].pid}")
                m.start()
                time.sleep(sleep_time)
                stats = m.get_stats()
                self.assertEqual(len(stats), 1, "Expected stats for one node")
            time.sleep(sleep_time)
            stats = m.get_stats()
            self.assertEqual(len(stats), 0, "Expected no stats after node termination")

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

import datetime
import psutil
import threading
import typing
import time
import sys
from contextlib import contextmanager
from cortex.config import CRTXEnvironment
from cortex.db import TemporalCRTX
from cortex.db.entities import NodeResourceUtilization
from cortex.types import NodeStats


class NodeMonitor:
    def __init__(self, pid: int, name: str):
        self.__pid = pid
        self.__name = name
        self.__process = psutil.Process(pid)

    @property
    def pid(self):
        return self.__pid

    @property
    def name(self):
        return self.__name

    @property
    def stats(self) -> NodeStats:
        try:
            with self.__process.oneshot():
                node_stats = NodeStats(
                    self.__name,
                    self.__pid,
                    self.__process.status(),
                    self.__process.cpu_percent(),
                    self.__process.memory_percent(),
                    self.__process.num_threads(),
                    self.__process.num_fds(),
                )
        except:
            node_stats = NodeStats(self.__name, -1, "terminated", -1, -1, -1, -1)
        return node_stats

    @staticmethod
    def to_entity(stats: NodeStats, robot, host) -> NodeResourceUtilization:
        msg_time = datetime.datetime.now()
        entity = NodeResourceUtilization(
            time=msg_time,
            msg_time=msg_time,
            robot=robot,
            node=stats.name,
            host=host,
            process=stats.name,
            status=stats.status,
            cpu_percent=stats.cpu_percent,
            memory_percent=stats.memory_percent,
            num_threads=stats.num_threads,
            num_fds=stats.num_fds,
        )
        return entity

    def __repr__(self):
        return (
            f"<NodeMonitor(pid={self.__pid}, name={self.__name}, stats={self.stats})>"
        )


class Monitor:
    """
    The Monitor class is used to monitor the resource utilization of ROS nodes running on a given host.

    To make this class generic, Monitor does not attempt to find the nodes running on the host. Instead, the user
    must provide the pids of the nodes to monitor. See cortex.ros1 and cortex.ros2 for distro-specific implementations.
    """

    def __init__(self, db_hostname=None, db_port=None, hz=None):
        env = CRTXEnvironment.local()
        self.__db_hostname = db_hostname if db_hostname else env.system.DB_HOSTNAME
        self.__db_port = db_port if db_port else env.system.DB_PORT
        self.__hostname = env.device.HOSTNAME
        self.__hz = hz if hz else env.system.MONITOR_HZ
        self.__robot = env.system.ROBOT
        self.__period = 1.0 / float(self.__hz)
        self.__node_monitors = []

        self.__next_update = time.time()

        # Use threading to start a thread to collect stats at the given frequency and publish them to the database
        self.__running = True
        self.__collect_stats_thread = threading.Thread(target=self.__collect_stats)
        self.__lock = threading.Lock()

        self.__db = TemporalCRTX(self.__db_hostname, self.__db_port)

    @property
    def is_running(self):
        return self.__running

    def start(self):
        """Start the Monitor. This will start a thread to collect stats at the given frequency and publish them to the database."""
        self.__collect_stats_thread.start()

    def stop(self, delay=3):
        """Stop the Monitor. This will stop the thread collecting stats and shut down the database.
        Note: This method will block until the database has been shut down. Stopping the monitor is final, and it cannot be restarted.

        Args:
            delay (int, optional): The number of seconds to wait before shutting down the database. Defaults to 3.
        """
        self.__running = False

        time.sleep(delay)

        if self.__collect_stats_thread.is_alive():
            self.__collect_stats_thread.join(delay)

        if self.__collect_stats_thread.is_alive():
            sys.exit()

        self.__collect_stats_thread = None
        self.__db.shutdown(block=True)

    def __collect_stats(self):
        while self.__running:
            self.__next_update += self.__period

            stats = self.get_stats()
            entities = [
                NodeMonitor.to_entity(stat, self.__robot, self.__hostname)
                for stat in stats
            ]
            self.__db.insert(entities)

            # Any nodes that have terminated should be removed from the list of nodes to monitor
            for stat in stats:
                if stat.pid == -1 or stat.status == "terminated":
                    self.remove_node(stat.name)

            time.sleep(max(0.0, self.__next_update - time.time()))

    def add_node(self, pid: int, name: str):
        """Add a node to the list of nodes to monitor. The node is identified by its PID and name.

        Args:
            pid (int): The process ID of the node to monitor
            name (str): The name of the node to monitor

        Raises:
            ValueError: If the PID is invalid or the node name already exists in the list of nodes to monitor
        """
        # If the PID is invalid, throw an error
        if pid <= 0:
            raise ValueError("Invalid PID")

        # Make sure the PID corresponds to a live process
        try:
            process = psutil.Process(pid)
            if process.status() == "terminated":
                raise ValueError("Invalid PID")
            process = None
        except psutil.NoSuchProcess:
            raise ValueError("Invalid PID")

        # If the node name already exists, throw an error
        found = [f for f in filter(lambda x: x.name == name, self.__node_monitors)]
        if len(found) > 0:
            raise ValueError(f"Node ({name}) already exists")

        self.__lock.acquire()
        self.__node_monitors.append(NodeMonitor(pid, name))
        self.__lock.release()

    def get_stats(self) -> typing.List[NodeStats]:
        """Return the utilization statistics for all nodes on this host. Uses multiprocessing to get the stats for all nodes in parallel."""
        if len(self.__node_monitors) == 0:
            return []

        self.__lock.acquire()
        stats = [n.stats for n in self.__node_monitors]
        self.__lock.release()

        return stats

    def remove_node(self, name: str):
        """Remove a node from the list of monitored nodes. The node is identified by its name."""
        self.__lock.acquire()
        for node in self.__node_monitors:
            if node.name == name:
                self.__node_monitors.remove(node)
        self.__lock.release()

    @staticmethod
    @contextmanager
    def at_rate(hz=1):
        """A context manager to run a Monitor at a given rate. This is useful for running a Monitor in a with statement."""
        m = Monitor(hz=hz)
        yield m
        if m.is_running:
            m.stop(delay=10)

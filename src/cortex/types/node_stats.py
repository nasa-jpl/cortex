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


class NodeStats:
    """
    A class to represent the statistics of a ROS node.

    Attributes
    ----------
    name : str
        The name of the node.
    pid : int
        The process ID of the node.
    status : str
        The status of the node.
    cpu_percent : float
        The CPU utilization of the node.
    memory_percent : float
        The memory utilization of the node.
    num_threads : int
        The number of threads running in the node.
    num_fds : int
        The number of file descriptors open in the node.
    """
    def __init__(self, name: str, pid: int, status: str, cpu_percent: float, memory_percent: float, num_threads: int, num_fds: int):
        self.__name = name
        self.__pid = pid
        self.__status = status
        self.__cpu_percent = cpu_percent
        self.__memory_percent = memory_percent
        self.__num_threads = num_threads
        self.__num_fds = num_fds

    @property
    def name(self):
        return self.__name

    @property
    def pid(self):
        return self.__pid

    @property
    def status(self):
        return self.__status

    @property
    def cpu_percent(self):
        return self.__cpu_percent

    @property
    def memory_percent(self):
        return self.__memory_percent

    @property
    def num_threads(self):
        return self.__num_threads

    @property
    def num_fds(self):
        return self.__num_fds

    @property
    def stats(self):
        return {
            "name": self.__name,
            "pid": self.__pid,
            "status": self.__status,
            "cpu_percent": self.__cpu_percent,
            "memory_percent": self.__memory_percent,
            "num_threads": self.__num_threads,
            "num_fds": self.__num_fds
        }

    def __repr__(self):
        return f"<NodeStats(name={self.__name}, pid={self.__pid}, status={self.__status}, cpu_percent={self.__cpu_percent}, memory_percent={self.__memory_percent}, num_threads={self.__num_threads}, num_fds={self.__num_fds})>"

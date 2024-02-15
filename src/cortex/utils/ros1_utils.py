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
import functools

import rosgraph
import time
from functools import lru_cache
from xmlrpc.client import ServerProxy

class ROS1Utils:
    def __init__(self, master_uri=None):
        self.__ros_master = rosgraph.Master('/cortex', master_uri)

    def get_ros_nodes(self) -> set:
        sys_state = self.__ros_master.getSystemState()
        nodes = []
        for state in sys_state:
            for topic, node in state:
                nodes.extend(node)
        return set(nodes)

    @functools.lru_cache(maxsize=256)
    def get_node_pids(self, nodes: set = None, ttl_hash=None) -> dict:
        """
        Get the process IDs of the ROS nodes.

        This function uses lru_cache to cache the PIDs in order to avoid unnecessary calls to the ROS master.
        In order to avoid stale cache data, simply pass a different value for the ttl_hash parameter.

        :param nodes: a set of ROS nodes
        :param master_uri: the URI of the ROS master
        :param ttl_hash: a hash to calculate the time-to-live of the cache
        :return:
        """
        # A trick to add expiration to the cache
        del ttl_hash

        print(f"Getting nodes (not from cache)")
        pids = {}

        if nodes is None:
            nodes = self.get_ros_nodes()

        for node in nodes:
            uri = self.__ros_master.lookupNode(node)
            proxy = ServerProxy(uri)
            pid = proxy.getPid()
            if pid[0] == 1:
                pids[node] = pid[2]
            else:
                print(f"Error getting PID for node {node}: {pid}")
        return pids

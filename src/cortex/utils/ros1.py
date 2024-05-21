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
import rospy
from xmlrpc.client import ServerProxy


class ROS1Utils:
    CALLER_ID = "/rosout"

    def __init__(self):
        pass

    @staticmethod
    def get_ros_nodes(master_uri=None) -> set:
        """
        Get a list of all running ROS nodes on the master.

        :param master_uri: the URI of the ROS master (e.g. http://localhost:11311)
        :return: a set of node names
        """
        sys_state = rosgraph.Master(ROS1Utils.CALLER_ID, master_uri).getSystemState()
        nodes = []
        for state in sys_state:
            for topic, node in state:
                nodes.extend(node)
        return set(nodes)

    @staticmethod
    @functools.lru_cache(maxsize=1)
    def get_node_pids(nodes: set = None, master_uri=None, hash=None) -> dict:
        """
        Get the process IDs of the given ROS nodes.

        This function uses lru_cache to cache the PIDs in order to avoid unnecessary calls to the ROS master.
        In order to avoid stale cache data, simply pass a different value for the ttl_hash parameter. Note
        that the cache will also be invalidated if the nodes or master_uri parameters change.

        :param nodes: a set of ROS nodes to get the PIDs for. If None, the function will get all nodes from the master.
        :param master_uri: the URI of the ROS master
        :param hash: a hash used to invalidate the cache (e.g. by ttl)
        :return:
        """
        rospy.loginfo(f"Getting Nodes/PIDs <master_uri={master_uri}, ttl_hash={hash}>")
        pids = {}

        if nodes is None:
            nodes = ROS1Utils.get_ros_nodes(master_uri)

        for node in nodes:
            try:
                uri = rosgraph.Master(ROS1Utils.CALLER_ID, master_uri).lookupNode(node)
                proxy = ServerProxy(uri)
                pid = proxy.getPid(ROS1Utils.CALLER_ID)
                if pid[0] == 1:
                    pids[node] = pid[2]
                else:
                    print(f"Error getting PID for node {node}: {pid}")
            except Exception as e:
                rospy.logerr(f"Error getting PID for node {node}: {e}")
                continue
        return pids

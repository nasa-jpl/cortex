#!/usr/bin/env python

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

import rospy
import time
from cortex.agents import Monitor
from cortex.config import CRTXEnvironment
from cortex.utils import ROS1Utils


def cache_ttl(seconds=10):
    """A function used to calculate the cache time-to-live hash."""
    return round(time.time() / seconds)


def main():
    # Use CRTX launch file to set node name and namespace
    rospy.init_node("monitor")

    env = CRTXEnvironment.local()
    monitor = Monitor(env.system.DB_HOSTNAME, env.system.DB_PORT)

    utils = ROS1Utils()
    pids = utils.get_node_pids(ttl_hash=cache_ttl())
    for node, pid in pids.items():
        monitor.add_node(pid, node)
    monitor.start()

    while not rospy.is_shutdown():
        try:
            new_pids = utils.get_node_pids(ttl_hash=cache_ttl())
        except ConnectionRefusedError:
            break

        # Find the difference between the two sets
        added = new_pids.keys() - pids.keys()
        removed = pids.keys() - new_pids.keys()

        if added or removed:
            print(f"Detecting changes in nodes: {added} {removed}")
            for node in added:
                try:
                    monitor.add_node(new_pids[node], node)
                except ValueError as e:
                    rospy.logerr(f"Error adding node {node}: {e}")
                    del new_pids[node]
            for node in removed:
                monitor.remove_node(node)

        pids = new_pids
        time.sleep(0.5)

    if monitor.is_running:
        monitor.stop()


if __name__ == "__main__":
    main()

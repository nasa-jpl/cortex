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

from cortex.agents import CRTXMonitor
from cortex.config import CRTXEnvironment
from cortex.utils import ROS1Utils, CacheUtils


class ROS1Monitor(CRTXMonitor):
    def __init__(self, node_name="~", hz=1.0, db_hostname=None, db_port=None):
        rospy.init_node(node_name)

        super().__init__(
            db_hostname=db_hostname,
            db_port=db_port,
            hz=hz,
        )

    def spin(self):
        # Get an initial list of ROS nodes and their PIDs
        pids = ROS1Utils.get_node_pids(hash=CacheUtils.expire_by_ttl())
        for node, pid in pids.items():
            try:
                self.add_node(pid, node)
            except Exception as e:
                rospy.logerr(f"Error adding node (ignoring) {node}: {e}")
        self.start()

        while not rospy.is_shutdown():
            try:
                new_pids = ROS1Utils.get_node_pids(hash=CacheUtils.expire_by_ttl())
            except ConnectionRefusedError:
                break

            # Find the difference between the two sets
            added = new_pids.keys() - pids.keys()
            removed = pids.keys() - new_pids.keys()

            if added or removed:
                rospy.loginfo(
                    f"Detecting changes in nodes... ({len(added)} added, {len(removed)} removed)"
                )
                for node in added:
                    try:
                        self.add_node(new_pids[node], node)
                    except Exception as e:
                        rospy.logerr(f"Error adding node (ignoring) {node}: {e}")
                        del new_pids[node]
                for node in removed:
                    self.remove_node(node)

            pids = new_pids
            rospy.sleep(0.5)

        if self.is_running:
            self.stop()


def main():
    env = CRTXEnvironment.local()
    monitor = ROS1Monitor(
        node_name="monitor",
        hz=env.MONITOR_HZ,
        db_hostname=env.system.DB_HOSTNAME,
        db_port=env.system.DB_PORT,
    )
    monitor.spin()


if __name__ == "__main__":
    main()

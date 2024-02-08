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
from cortex.agents import Monitor
from cortex.config import CRTXEnvironment


def main():
    # Use CRTX launch file to set node name and namespace
    rospy.init_node("~")

    env = CRTXEnvironment.local()

    monitor = Monitor(env.system.DB_HOSTNAME, env.system.DB_PORT)

    while not rospy.is_shutdown():
        # Get a list of nodes that are running on this device
        ## Handle nodes that are not in the monitor
        ### Handle nodes that are in the list but not in the monitor
        #### Get the PID for the node
        #### Add it to the monitor
        ### Handle nodes that are in the monitor but not in the list
        #### Remove the node from the monitor

        # Start the monitor if it isn't already running
        if not monitor.is_running:
            monitor.start()

        # Wait for a period of time before checking again
        pass

    if monitor.is_running:
        monitor.stop()


if __name__ == "__main__":
    main()

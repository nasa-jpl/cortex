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
import typing
from cortex.config import CRTXEnvironment, WorkerConfig, BasicWorkerConfig
from cortex.agents import CRTXWorker


class ROS1Worker:
    def __init__(self, node_name, global_args):
        rospy.init_node(node_name)
        self.workers: typing.List[CRTXWorker] = []

        # Parse YAML file for a list of worker configurations
        for config in WorkerConfig.get_basic_worker_config(global_args):
            worker = CRTXWorker(BasicWorkerConfig(config))
            rospy.Subscriber(
                worker.topic, worker.data_class, worker.callback, queue_size=1
            )
            self.workers.append(worker)
            print(f"Worker started: {worker}")

    def spin(self):
        rospy.spin()


def main():
    env = CRTXEnvironment.local()
    global_args = dict(robot=env.system.ROBOT, host=env.device.HOSTNAME)

    print("Starting ROS1 worker...")
    worker = ROS1Worker(node_name="~", global_args=global_args)
    worker.spin()


if __name__ == "__main__":
    main()

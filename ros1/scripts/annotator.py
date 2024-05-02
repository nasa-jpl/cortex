#!/usr/bin/env python
import datetime

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
from cortex.agents import CRTXAnnotator
from cortex.config import CRTXEnvironment

try:
    from cortex.msg import Annotation
except ImportError as e:
    raise e


class ROS1Annotator(CRTXAnnotator):
    """
    CORTEX Annotation Service for ROS1.
    Provides endpoints for annotating data, system events, etc. and stores them in the CORTEX database.
    """

    def __init__(self, node_name, host, robot):
        rospy.init_node(node_name)

        super().__init__(host, robot)

        self.__service = rospy.Subscriber("~annotate", Annotation, self.__callback)

    def __callback(self, msg):
        # Accept the message and process it (add annotation to database)
        annotation = self.annotation(msg)
        self.db.insert(annotation)

    def spin(self):
        rospy.spin()


def main():
    env = CRTXEnvironment.local()
    annotator = ROS1Annotator("~", env.device.HOSTNAME, env.system.ROBOT)
    annotator.spin()


if __name__ == "__main__":
    main()

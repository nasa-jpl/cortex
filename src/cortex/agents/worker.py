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


import time
import typing
from cortex.db import TemporalCRTX
from cortex.config.workers import *


class Worker:
    def __init__(self, config: typing.Union[BasicWorkerConfig]):
        self.__topic = config.topic
        self.__data_type = config.data_type
        self.__msg_pkg = config.msg_pkg
        self.__hz = config.hz
        self.__db: TemporalCRTX = config.db
        self.__transform = config.transform
        self.__global_args = config.global_args
        self.__change_only = config.change_only
        self.__change_field = config.change_field

        self.__desired_delta = float(1 / config.hz) if config.hz is not None else None
        self.__previous_msg = None
        self.__previous_time = None

    def callback(self, msg, kwargs):
        # This callback should be called by the ROS subscriber
        if self.__should_process(msg):
            current_time = time.time()
            entities = self.__transform(msg, **self.__global_args, **kwargs)
            self.__db.insert(entities)

            if self.__change_only:
                self.__previous_msg = msg

            self.__previous_time = current_time

    def __should_process(self, msg):
        # Note that change_only effectively overrides the behavior of desired_hz since we assume ALL changes should be processed
        if self.__change_only:
            # Always process the first message
            if self.__previous_msg is None:
                return True

            if getattr(self.__previous_msg, self.__change_field) == getattr(
                msg, self.__change_field
            ):
                # Do not process if change_only is enabled and the change_field has not changed
                return False
            else:
                # Process if change_only is enabled and the change_field has changed
                return True

        # If desired_hz is not set, process every message
        if self.__hz is None or self.__desired_delta is None:
            return True

        # Process if the desired time delta has passed
        if (
            self.__previous_time is not None
            and self.__desired_delta is not None
            and (time.time() - self.__previous_time) < self.__desired_delta
        ):
            return False

        # Otherwise, process
        return True

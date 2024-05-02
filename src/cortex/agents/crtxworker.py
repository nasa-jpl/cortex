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
from datetime import datetime
from typing import Union

from cortex.config import BasicWorkerConfig
from cortex.db import TemporalCRTX


class CRTXWorker:
    """A worker class for processing ROS messages and inserting them into the CORTEX database.

    Args:
        config: The configuration for the worker (currently only supports `BasicWorkerConfig`).
    """
    def __init__(self, config: Union[BasicWorkerConfig]):
        self.__topic = config.topic
        self.__data_type = config.data_type
        self.__msg_pkg = config.msg_pkg
        self.__hz = config.hz
        self.__transform = config.transform
        self.__global_args = config.global_args
        self.__change_fields = config.change_fields
        self.__desired_delta = float(1 / config.hz) if config.hz is not None else None
        self.__previous_msg = None
        self.__previous_time = None
        self.__db = TemporalCRTX()

    @property
    def topic(self):
        """The topic that the worker is subscribed to."""
        return self.__topic

    @property
    def data_class(self):
        """The class of the data type being processed on the topic."""
        msg_pkg = __import__(self.__msg_pkg, fromlist=[self.__data_type])
        return getattr(msg_pkg, self.__data_type)

    def callback(self, msg, **kwargs):
        """The callback function for the worker. Processes the message and inserts it into the database."""
        if self.__should_process(msg):
            current_time = time.time()

            if hasattr(msg, "__slots__") and "header" in msg.__slots__:
                # ROS1 message
                msg_time = datetime.fromtimestamp(msg.header.stamp.to_sec())
            else:
                # No header
                msg_time = datetime.fromtimestamp(current_time)

            kwargs.update(
                {"time": datetime.fromtimestamp(current_time), "msg_time": msg_time}
            )

            # Transform the message and insert it into the database
            entities = self.__transform(msg, **self.__global_args, **kwargs)
            self.__db.insert(entities)

            if self.__change_fields:
                self.__previous_msg = msg
            self.__previous_time = current_time

    def __should_process(self, msg):
        """Determines if the message should be processed and inserted based on the worker's configuration."""
        # Note that any entry in change_fields effectively overrides the hz setting
        if self.__change_fields:
            # Always process the first message
            if self.__previous_msg is None:
                return True

            for change_field in self.__change_fields:
                # Process if any of the tracked fields have changed
                if getattr(msg, change_field) != getattr(
                    self.__previous_msg, change_field
                ):
                    return True
            # Otherwise, if we only process on change and no changes have been detected, return False
            return False

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

    def __del__(self):
        if getattr(self, "__db", None):
            try:
                self.__db.shutdown(block=True)
            except:
                pass

    def __repr__(self):
        return f"CRTXWorker(topic={self.__topic}, data_type={self.__data_type}, msg_pkg={self.__msg_pkg}, hz={self.__hz}, change_fields={self.__change_fields}, global_args={self.__global_args})"

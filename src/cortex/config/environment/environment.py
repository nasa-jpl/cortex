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
import typing

from .device import DeviceEnvironment
from .system import SystemEnvironment


class CRTXEnvironment:
    """
    The Environment class is used to set/get environment variables for the CORTEX system.

    In every configuration, there will be a single device responsible for initiating the CORTEX SystemEnvironment.
    The SystemEnvironment will then be made available to all other devices in the system. Once the devices
    have the SystemEnvironment, they can use it to set their own unique DeviceEnvironment.
    """
    def __init__(self):
        # Define the environment variabels we care about. We will determine the correct method
        # to retrieve them later.
        self.system: typing.Union[SystemEnvironment, None] = None
        self.device: typing.Union[DeviceEnvironment, None] = None

    def __iter__(self):
        for env in [self.system, self.device]:
            for attr in dir(env):
                if not attr.startswith('__') and not callable(getattr(env, attr)):
                    yield attr, getattr(env, attr)

    def __getitem__(self, item):
        # item will be one of the variables set in either SystemEnvironment or DeviceEnvironment
        for env in [self.system, self.device]:
            if hasattr(env, item):
                return getattr(env, item)

    def __getattr__(self, item):
        return self.__getitem__(item)

    @staticmethod
    def local() -> 'CRTXEnvironment':
        env = CRTXEnvironment()
        env.system = SystemEnvironment.local()
        env.device = DeviceEnvironment.local()
        return env

    def __repr__(self):
        return f"System Environment:\n{self.system}\nDevice Environment:\n{self.device}"

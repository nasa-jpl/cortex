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

import os


class DeviceEnvironment:
    def __init__(self):
        self.HOSTNAME = DeviceEnvironment.get_hostname()

    @staticmethod
    def local() -> 'DeviceEnvironment':
        device_env = DeviceEnvironment()

        # Loop over the class attributes and set them to the environment variable if it exists, otherwise their default
        for attr in dir(device_env):
            if not attr.startswith('__') and not callable(getattr(device_env, attr)):
                setattr(device_env, attr, os.environ.get(attr, getattr(device_env, attr)))

        return device_env

    @staticmethod
    def get_hostname() -> str:
        hostname = os.environ.get('HOSTNAME')
        if hostname is None:
            # Use the system hostname
            hostname = os.uname().nodename
        return hostname

    # Method to get dictionary from this class
    def __iter__(self):
        for attr in dir(self):
            if not attr.startswith('__') and not callable(getattr(self, attr)):
                yield attr, getattr(self, attr)

    def __repr__(self):
        repr = ''

        # Loop over the class attributes and add them to the repr
        for attr in dir(self):
            if not attr.startswith('__') and not callable(getattr(self, attr)):
                repr += f"\t{attr}: {getattr(self, attr)}\n"

        return repr

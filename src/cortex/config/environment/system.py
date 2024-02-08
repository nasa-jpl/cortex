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
import datetime


class SystemEnvironment:
    def __init__(self):
        # Set any default values here, otherwise leave them as None
        self.DB_HOSTNAME = '127.0.0.1'
        self.DB_PORT = '5432'
        self.MONITOR_HZ = '1.0'
        self.RUN_NUMBER = '0'
        self.RUN_START_TIME = datetime.datetime.now().isoformat()
        self.RUN_LABEL = 'default'
        self.RUN_DESCRIPTION = ''
        self.ROBOT = 'Test'

    """A method to retrieve environment variables from the system (e.g. os.environ, etc.)"""
    @staticmethod
    def local() -> 'SystemEnvironment':
        sysenv = SystemEnvironment()

        # Loop over the class attributes and set them to the environment variable if it exists, otherwise their default
        for attr in dir(sysenv):
            if not attr.startswith('__') and not callable(getattr(sysenv, attr)):
                setattr(sysenv, attr, os.environ.get(attr, getattr(sysenv, attr)))

        return sysenv

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

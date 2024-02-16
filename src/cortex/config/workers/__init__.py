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
import yaml

from .config import *

WORKER_CONFIG_PATH = os.path.dirname(__file__)


class WorkerConfig:
    """A class to read and store worker configurations from a YAML file."""

    @staticmethod
    def get_basic_worker_config(global_args=None):
        config = WorkerConfig.get_config("basic")
        if global_args:
            for cfg in config:
                cfg["global_args"] = global_args
        return config

    @staticmethod
    def get_config(config_type: str):
        # Get config_type.yaml file path
        config_file = os.path.join(WORKER_CONFIG_PATH, config_type + ".yaml")
        print(f"Config file: {config_file}")
        # Read the file
        with open(config_file, "r") as file:
            return yaml.safe_load(file)

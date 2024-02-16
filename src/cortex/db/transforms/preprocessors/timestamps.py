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

from datetime import datetime
import typing


def parse_timestamps(fields: typing.List[str], data: typing.List[dict]):
    # For each element in data, convert the timestamp to a datetime object
    for row in data:
        for field in fields:
            if field in row:
                try:
                    # Try a ROS1-specific conversion
                    import genpy

                    if isinstance(row[field], genpy.Time):
                        row[field] = datetime.fromtimestamp(row[field].to_sec())
                    elif isinstance(row[field], genpy.Duration):
                        row[field] = row[field].to_sec()
                except:
                    row[field] = datetime.fromtimestamp(row[field])
    return data


def test():
    pass

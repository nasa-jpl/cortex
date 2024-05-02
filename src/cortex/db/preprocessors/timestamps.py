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
                    else:
                        print(f"{field} is neither a Time nor a Duration object")
                        row[field] = datetime.fromtimestamp(row[field])

                    print(f"Converted {field} to datetime object ({row[field]})")
                except:
                    print(f"ROS1 conversion not available... trying generic conversion")
                    row[field] = datetime.fromtimestamp(row[field])

                try:
                    row[field] = datetime.fromtimestamp(row[field])
                except:
                    pass

            else:
                print(f"Field {field} not found in row {row}")
    return data


def test():
    pass

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

import typing


def flatten(fields, data: typing.List[dict]):
    # Assumes that the data is an object with multiple arrays
    # e.g. data = [ {a: [1,2,3], b: [4,5,6,7], c: [8,9,10,11]} ]
    # The fields parameter is a list of the fields to flatten
    # e.g. fields = ['a', 'b']
    # The result will be a list of objects with the flattened fields, whose length is the length of the shortest field
    # e.g. result = [{a: 1, b: 4}, {a: 2, b: 5}, {a: 3, b: 6}]

    results = []
    for d in data:
        # Get the length of the shortest field to ensure that we don't go out of bounds
        length = min([len(d[field]) for field in fields])

        # Get a list of fields that are not lists and not in the fields list
        keys = d.keys()
        other_fields = [
            field
            for field in keys
            if field not in fields and not isinstance(d[field], list)
        ]

        for i in range(length):
            obj = {}

            # Add the other fields to the object
            for field in other_fields:
                obj[field] = d[field]

            for field in fields:
                obj[field] = d[field][i]
            results.append(obj)
    return results

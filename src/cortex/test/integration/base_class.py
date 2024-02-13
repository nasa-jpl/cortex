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
import unittest


class IntegrationBase(unittest.TestSuite):
    test_name = None

    def __init__(self):
        super(IntegrationBase, self).__init__()

    def run(self, result, debug=False):
        print("Running integration test: " + self.test_name)
        super(IntegrationBase, self).run(result, debug)
        return result

    def __init_subclass__(cls, **kwargs):
        if not getattr(cls, 'test_name', None):
            raise AttributeError("[IntegrationBase]: derived classes must specify a test_name attribute.")

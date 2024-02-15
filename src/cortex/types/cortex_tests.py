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

import time
import unittest
from datetime import datetime


class CRTXTest:
    """A class to represent test results."""
    id = None
    desc = None
    start: datetime = None
    end: datetime = None
    status = None
    result = None
    error = None
    failure = None
    tags = None

    def __repr__(self):
        s = ''
        if self.status == 'SUCCESS':
            # Set the color to GREEN
            s = f'\033[92m[{self.status.upper()}] ({self.id})\033[0m'
        elif self.status == 'FAILURE':
            # Set the color to RED
            s = f'\033[91m[{self.status.upper()}] ({self.id})\033[0m'
            s += f'\n\tReason: {self.failure}'
        elif self.status == 'ERROR':
            # Set the color to RED
            s = f'\033[91m[{self.status.upper()}] ({self.id})\033[0m'
            s += f'\n\tError: {self.error}'
        s += f'\n\tStart: {self.start.strftime("%Y-%m-%d %H:%M:%S.%f")}'
        s += f'\n\tEnd: {self.end.strftime("%Y-%m-%d %H:%M:%S.%f")}'
        s += f'\n\tDuration: {self.end - self.start}'
        if self.desc:
            s += f'\n\tDescription: {self.desc}'
        return s


class CRTXTestRunner(unittest.TextTestRunner):
    """A test runner that publishes test results to the CORTEX database."""
    def __init__(self, *args, **kwargs):
        # Make sure the base test runner uses our custom test result class
        kwargs['resultclass'] = CRTXTestResult
        super(CRTXTestRunner, self).__init__(*args, **kwargs)

    def run(self, test):
        return super(CRTXTestRunner, self).run(test)


class CRTXTestResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super(CRTXTestResult, self).__init__(*args, **kwargs)
        self.test_results = []
        self.test = CRTXTest()

        from cortex.db import TemporalCRTX
        from cortex.config import CRTXEnvironment

        self.db = TemporalCRTX(logging=False)
        self.env = CRTXEnvironment.local()

    def addError(self, test, err):
        self.test.status = 'ERROR'
        self.test.error = err
        super().addError(test, err)

    def addSuccess(self, test):
        self.test.status = 'SUCCESS'
        super(CRTXTestResult, self).addSuccess(test)

    def addFailure(self, test, err):
        self.test.status = 'FAILURE'
        self.test.failure = err[1]
        super(CRTXTestResult, self).addFailure(test, err)

    def startTest(self, test):
        test_path = test.id().split('.')
        self.test.id = f"{test_path[2].upper()}: {test_path[-2]}.{test_path[-1]}()"
        self.test.desc = test.shortDescription()
        self.test.start = datetime.now()
        super().startTest(test)

    def stopTest(self, test):
        self.test.end = datetime.now()
        self.test_results.append(self.test)
        self.__publish(self.test)

        self.test = CRTXTest()
        super().stopTest(test)

    def stopTestRun(self):
        # Give the database time to insert the most recent test results before shutting down
        time.sleep(2)
        self.db.shutdown(block=True)

    def __publish(self, test: CRTXTest):
        from cortex.db.entities import Annotation

        level = 'INFO' if test.status == 'SUCCESS' else 'ERROR'
        message = test.desc if test.status == 'SUCCESS' else f"[Reason: {test.failure}]\n{test.desc}"
        if message is None:
            message = f"Test description not specified in the {test.id} docstring."

        annotation = Annotation(
            time=test.start,
            msg_time=test.start,
            robot=self.env.system.ROBOT,
            host=self.env.device.HOSTNAME,
            label=test.id,
            message=message,
            tags=['test', test.status],
            level=level,
            end_time=test.end,
        )

        self.db.insert([annotation])

    def __repr__(self):
        s = ''
        for test in self.test_results:
            s += f"{test}\n"
        return s

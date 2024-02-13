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
from cortex.db import TemporalCRTX
from cortex.db.entities import Robot


class TestTemporalCRTX(unittest.TestCase):
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_connection(self):
        """Make sure we can connect to the database"""
        temporal = TemporalCRTX(logging=False)
        assert temporal is not None
        temporal.shutdown(block=True)

    def test_insertion(self):
        """Make sure we can insert a record into the database"""
        temporal = TemporalCRTX(batch_timeout=0.5, logging=False)
        robot = Robot(name='R2D2', description='Astromech droid')
        temporal.insert(robot)

        # Wait a few seconds for insertion
        import time
        time.sleep(2)

        # Verify that it was inserted then delete it
        with temporal.get_session() as session:
            r = session.query(Robot).filter_by(name='R2D2').first()
            assert r is not None
            session.delete(r)
            session.commit()

        # Verify that it was deleted by trying to query again
        with temporal.get_session() as session:
            r = session.query(Robot).filter_by(name='R2D2').first()
            assert r is None

        temporal.shutdown(block=True)

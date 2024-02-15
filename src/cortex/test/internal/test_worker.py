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
import yaml
from datetime import datetime
from cortex.db import TemporalCRTX
from cortex.db.entities import JointStatesActual
from cortex.config import BasicWorkerConfig
from cortex.agents import Worker


class TestBasicWorker(unittest.TestCase):
    def setUp(self):
        config_yaml = """
        - topic: /joint_states
          data_type: JointState
          msg_pkg: sensor_msgs.msg
          hz: 5
          target: joint_states_actual
          preprocessors:
            - flatten:
                fields:
                  - name
                  - position
                  - velocity
                  - effort
            - rename:
                mappings:
                    name: actuator
            """
        config_obj = yaml.load(config_yaml, Loader=yaml.FullLoader)[0]
        self.global_args = dict(robot="Test Robot", host="dev1")

        config_obj['global_args'] = self.global_args

        self.config = BasicWorkerConfig(config_obj)
        self.right_now = datetime.now().isoformat()
        self.data = dict(
            time=self.right_now,
            msg_time=self.right_now,
            name=["js1", "js2", "js3", "js4"],
            velocity=[5, 6, 7],
            position=[8, 9, 10, 11, 12],
            effort=[13, 14, 15, 16],
        )
        self.db = TemporalCRTX(logging=False)

    def tearDown(self):
        with self.db.get_session() as session:
            # Remove all entities with robot name "Test Robot"
            session.query(JointStatesActual).where(JointStatesActual.robot == "Test Robot").delete()
            session.commit()
        self.db.shutdown(block=True)

    def test_basic_config(self):
        """Verify that the basic worker config object can be created and that it can transform data into target class rows."""

        # Transform the flattened data into target class rows
        entities = self.config.transform(self.data, **self.global_args)
        self.assertEqual(len(entities), 3, "Flattening should have created 3 entities.")

        # Verify that the entities are in the correct format
        for i in range(3):
            entity = entities[i]
            self.assertEqual(entity.robot, "Test Robot", "Robot name should be Test Robot")
            self.assertEqual(entity.time, self.right_now, f"time should be {self.right_now}")
            self.assertEqual(entity.msg_time, self.right_now, f"msg_time should be {self.right_now}")
            self.assertEqual(entity.actuator, self.data.get("name")[i], f"actuator should be {self.data.get('name')[i]}")
            self.assertEqual(entity.velocity, self.data.get("velocity")[i], f"velocity should be {self.data.get('velocity')[i]}")
            self.assertEqual(entity.position, self.data.get("position")[i], f"position should be {self.data.get('position')[i]}")
            self.assertEqual(entity.effort, self.data.get("effort")[i], f"effort should be {self.data.get('effort')[i]}")

    def test_basic_worker_sample_rate(self):
        """Verify that the basic worker can be created and that it obeys the specified sample rate."""
        worker = Worker(self.config)

        for i in range(5):
            # Create random data for position, velocity, and effort
            self.data['position'] = [i, i+1, i+2, i+3, i+4]
            self.data['velocity'] = [i+5, i+6, i+7]
            self.data['effort'] = [i+8, i+9, i+10, i+11]
            worker.callback(self.data)
            time.sleep(1 / self.config.hz)

        # Give the database time to process the data
        time.sleep(2)

        # Verify that the correct number of entities were created (should be 5 * 3 = 15)
        with self.db.get_session() as session:
            states = session.query(JointStatesActual).where(JointStatesActual.robot == "Test Robot").all()
            self.assertEqual(len(states), 15, "There should be 15 entities in the database.")

            # Remove them from the DB
            for state in states:
                session.delete(state)
            session.commit()

        # Now let's do it again, but this time we will send them immediately (hence there should only be 3 entities)
        for i in range(5):
            # Create random data for position, velocity, and effort
            self.data['position'] = [i, i+1, i+2, i+3, i+4]
            self.data['velocity'] = [i+5, i+6, i+7]
            self.data['effort'] = [i+8, i+9, i+10, i+11]
            worker.callback(self.data)

        # Give the database time to process the data
        time.sleep(2)

        # Since the data was sent faster than the sample rate, there should only be 3 entities
        with self.db.get_session() as session:
            states = session.query(JointStatesActual).where(JointStatesActual.robot == "Test Robot").all()
            self.assertEqual(len(states), 3, "There should be 3 entities in the database.")

            # Remove them from the DB
            for state in states:
                session.delete(state)
            session.commit()

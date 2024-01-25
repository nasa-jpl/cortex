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

from functools import partial


class BasicWorkerConfig:
    def __init__(self, config: dict):
        self.topic = config["topic"]
        self.db = config["db"]
        self.data_type = config["data_type"]
        self.msg_pkg = config["msg_pkg"]
        self.hz = config["hz"]
        self.target = config["target"]
        self.transforms = config["transforms"]
        self.global_args = config.get("global_args", {})
        self.preprocessors = self.get_preprocessors(config["preprocessors"])
        self.change_only = config.get("change_only", False)
        self.change_field = config.get("change_field", None)
        self.target_class = self.get_target_class()

    def get_target_class(self):
        # Retrieve the SQLAlchemy class for the target table from cortex.db.entities
        # Use the target name to get the class. Try using ts_{target} first, then rel_{target}
        # The __tablename__ attribute is used to get the table name from the class

        # Get all the classes from cortex.db.entities
        from cortex.db.entities import tables

        options = [f"ts_{self.target}", f"rel_{self.target}"]
        for option in options:
            if option in tables:
                cls = tables[option]
                break
        else:
            raise ValueError(f"Could not find target class for {self.target}")
        return cls

    def get_preprocessors(self, preprocessor_config):
        preprocessors = []
        for preprocessor in preprocessor_config:
            # The name is the only key in the preprocessor dict
            name = list(preprocessor.keys())[0]

            # Retrieve preprocessor function from cortex.db.transforms.preprocessors
            preprocessor_func = getattr(
                __import__("cortex.db.transforms.preprocessors", fromlist=[name]), name
            )

            # The args are the value associated with the name key
            args = preprocessor[name]

            # Create a partial function with the args applied to it using functools
            preprocessors.append(partial(preprocessor_func, **args))

        return preprocessors

    def transform(self, rows, **kwargs):
        # rows: a list of data that will be transformed into rows in the target table
        # target_class: the SQLAlchemy class for the target table
        # The self.transforms contains a map that maps the fields in the data to the columns in the target table
        # e.g. self.transforms = [ {name: actuator}, {position: position}, {velocity: velocity}, {effort: effort} ]
        # target_class has fields actuator, position, velocity, effort
        # So we need to map the data fields to the target_class fields
        entities = []

        # Apply the preprocessors to the rows
        for preprocessor in self.preprocessors:
            rows = preprocessor(data=rows)

        for row in rows:
            # Map the data fields to the target_class fields
            data = {}
            for transform in self.transforms:
                key, value = list(transform.items())[0]
                data[value] = row[key]

            # Add the kwargs to the data if the target_class has those fields
            for key, value in kwargs.items():
                if hasattr(self.target_class, key):
                    data[key] = value

            # Create a new target_class with the args from the row
            entity = self.target_class(**data)
            entities.append(entity)

        return entities


def test():
    # Get basic worker config from YAML file
    import yaml
    from datetime import datetime

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
  transforms:
    - time: time
    - msg_time: msg_time
    - name: actuator
    - position: position
    - velocity: velocity
    - effort: effort
    """

    config = yaml.load(config_yaml, Loader=yaml.FullLoader)[0]

    # Create basic worker config object
    basic_worker_config = BasicWorkerConfig(config)

    right_now = datetime.now()
    data = dict(
        time=right_now,
        msg_time=right_now,
        name=["js1", "js2", "js3", "js4"],
        velocity=[5, 6, 7],
        position=[8, 9, 10, 11, 12],
        effort=[13, 14, 15, 16],
    )

    global_args = dict(robot="Test Robot", host="eels-dev1")

    # Transform the flattened data into target class rows
    entities = basic_worker_config.transform(data, **global_args)
    print(f"Entities: ")
    print(entities)

    from cortex.db import TemporalCRTX

    db = TemporalCRTX()
    with db.get_session() as session:
        session.add_all(entities)
        session.commit()
    print(f"Done")

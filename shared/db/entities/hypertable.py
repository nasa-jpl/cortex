# Path: shared/db/entities/hypertable.py
# This file is automatically generated by CORTEX/utils/table_generator.py

# Copyright (c) 2024 Jet Propulsion Laboratory. All rights reserved.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#
#       https://www.apache.org/licenses/LICENSE-2.0
#
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

from sqlalchemy import Column, Integer, String, DateTime, Index, ARRAY, ForeignKey, DOUBLE_PRECISION
from sqlalchemy.orm import declarative_base

Base = declarative_base()


class Environment(Base):
    __tablename__ = 'ts_environment'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    host = Column(String, nullable=False)
    robot = Column(String, nullable=False)
    key = Column(String, nullable=False)
    value = Column(String, nullable=False)

    def __repr__(self):
        return f"""<Environment(time='{self.time}', msg_time='{self.msg_time}', host='{self.host}', robot='{self.robot}', key='{self.key}', value='{self.value}')>"""


Index('idx_environment_host', Environment.host)
Index('idx_environment_key', Environment.key)


class Annotation(Base):
    __tablename__ = 'ts_annotation'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    host = Column(String, nullable=False)
    label = Column(String, nullable=False)
    message = Column(String, nullable=False)
    tags = Column(ARRAY(String), nullable=True, )
    level = Column(String, nullable=True, )
    end_time = Column(DateTime(timezone=True), nullable=True, )

    def __repr__(self):
        return f"""<Annotation(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', host='{self.host}', label='{self.label}', message='{self.message}', tags='{self.tags}', level='{self.level}', end_time='{self.end_time}')>"""


Index('idx_annotation_robot_label', Annotation.robot, Annotation.label)

class Event(Base):
    __tablename__ = 'ts_event'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    host = Column(String, nullable=False)
    label = Column(String, nullable=False)
    message = Column(String, nullable=False)
    tags = Column(ARRAY(String), nullable=True, )
    level = Column(String, nullable=True, )
    end_time = Column(DateTime(timezone=True), nullable=True, )

    def __repr__(self):
        return f"""<Event(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', host='{self.host}', label='{self.label}', message='{self.message}', tags='{self.tags}', level='{self.level}', end_time='{self.end_time}')>"""


Index('idx_event_robot_label', Event.robot, Event.label)

class Heartbeat(Base):
    __tablename__ = 'ts_heartbeat'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    host = Column(String, nullable=False)
    label = Column(String, nullable=False)
    message = Column(String, nullable=False)
    tags = Column(ARRAY(String), nullable=True, )
    level = Column(String, nullable=True, )
    end_time = Column(DateTime(timezone=True), nullable=True, )

    def __repr__(self):
        return f"""<Heartbeat(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', host='{self.host}', label='{self.label}', message='{self.message}', tags='{self.tags}', level='{self.level}', end_time='{self.end_time}')>"""


Index('idx_heartbeat_robot_label', Heartbeat.robot, Heartbeat.label)


class Metric(Base):
    __tablename__ = 'ts_metric'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    name = Column(String, nullable=False)
    real = Column(DOUBLE_PRECISION, nullable=True, )
    str = Column(String, nullable=True, )

    def __repr__(self):
        return f"""<Metric(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', name='{self.name}', real='{self.real}', str='{self.str}')>"""




class Parameter(Base):
    __tablename__ = 'ts_parameter'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    name = Column(String, nullable=False)
    real = Column(DOUBLE_PRECISION, nullable=True, )
    str = Column(String, nullable=True, )

    def __repr__(self):
        return f"""<Parameter(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', name='{self.name}', real='{self.real}', str='{self.str}')>"""





class NodeResourceUtilization(Base):
    __tablename__ = 'ts_node_resource_utilization'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    node = Column(String, nullable=False)
    host = Column(String, nullable=False)
    process = Column(String, nullable=False)
    status = Column(String, nullable=False)
    cpu_percent = Column(DOUBLE_PRECISION, nullable=False)
    memory_percent = Column(DOUBLE_PRECISION, nullable=False)
    num_threads = Column(Integer, nullable=False)
    num_fds = Column(Integer, nullable=False)

    def __repr__(self):
        return f"""<NodeResourceUtilization(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', node='{self.node}', host='{self.host}', process='{self.process}', status='{self.status}', cpu_percent='{self.cpu_percent}', memory_percent='{self.memory_percent}', num_threads='{self.num_threads}', num_fds='{self.num_fds}')>"""


Index('idx_node_resource_utilization_robot', NodeResourceUtilization.robot)


class JointStatesActual(Base):
    __tablename__ = 'ts_joint_states_actual'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    actuator = Column(String, nullable=False)
    position = Column(DOUBLE_PRECISION, nullable=False)
    velocity = Column(DOUBLE_PRECISION, nullable=False)
    effort = Column(DOUBLE_PRECISION, nullable=False)

    def __repr__(self):
        return f"""<JointStatesActual(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', actuator='{self.actuator}', position='{self.position}', velocity='{self.velocity}', effort='{self.effort}')>"""


Index('idx_joint_states_actual_robot', JointStatesActual.robot)
Index('idx_joint_states_actual_robot_actuator', JointStatesActual.robot, JointStatesActual.actuator)

class JointStatesCommanded(Base):
    __tablename__ = 'ts_joint_states_commanded'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    actuator = Column(String, nullable=False)
    position = Column(DOUBLE_PRECISION, nullable=False)
    velocity = Column(DOUBLE_PRECISION, nullable=False)
    effort = Column(DOUBLE_PRECISION, nullable=False)

    def __repr__(self):
        return f"""<JointStatesCommanded(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', actuator='{self.actuator}', position='{self.position}', velocity='{self.velocity}', effort='{self.effort}')>"""


Index('idx_joint_states_commanded_robot', JointStatesCommanded.robot)
Index('idx_joint_states_commanded_robot_actuator', JointStatesCommanded.robot, JointStatesCommanded.actuator)


class ForceTorqueSensor(Base):
    __tablename__ = 'ts_force_torque_sensor'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    actuator = Column(String, nullable=False)
    force = Column(ARRAY(DOUBLE_PRECISION), nullable=False)
    torque = Column(ARRAY(DOUBLE_PRECISION), nullable=False)

    def __repr__(self):
        return f"""<ForceTorqueSensor(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', actuator='{self.actuator}', force='{self.force}', torque='{self.torque}')>"""


Index('idx_force_torque_sensor_robot_actuator', ForceTorqueSensor.robot, ForceTorqueSensor.actuator)


class StateTransitions(Base):
    __tablename__ = 'ts_state_transitions'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    previous_state = Column(String, nullable=False)
    current_state = Column(String, nullable=False)
    state_enum = Column(Integer, nullable=False)
    state_string = Column(String, nullable=False)
    module = Column(String, nullable=True, )

    def __repr__(self):
        return f"""<StateTransitions(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', previous_state='{self.previous_state}', current_state='{self.current_state}', state_enum='{self.state_enum}', state_string='{self.state_string}', module='{self.module}')>"""


Index('idx_state_transitions_robot_current_state', StateTransitions.robot, StateTransitions.current_state)


class TopicStatistics(Base):
    __tablename__ = 'ts_topic_statistics'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    topic = Column(String, nullable=False)
    node_subscriber = Column(String, nullable=False)
    node_publisher = Column(String, nullable=False)
    start_time = Column(DateTime(timezone=True), nullable=False)
    end_time = Column(DateTime(timezone=True), nullable=False)
    dropped_messages = Column(Integer, nullable=False)
    delivered_messages = Column(Integer, nullable=False)
    traffic = Column(Integer, nullable=False)
    period_mean = Column(DOUBLE_PRECISION, nullable=False)
    period_stddev = Column(DOUBLE_PRECISION, nullable=False)
    period_max = Column(DOUBLE_PRECISION, nullable=False)
    stamp_age_mean = Column(DOUBLE_PRECISION, nullable=False)
    stamp_age_stddev = Column(DOUBLE_PRECISION, nullable=False)
    stamp_age_max = Column(DOUBLE_PRECISION, nullable=False)

    def __repr__(self):
        return f"""<TopicStatistics(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', topic='{self.topic}', node_subscriber='{self.node_subscriber}', node_publisher='{self.node_publisher}', start_time='{self.start_time}', end_time='{self.end_time}', dropped_messages='{self.dropped_messages}', delivered_messages='{self.delivered_messages}', traffic='{self.traffic}', period_mean='{self.period_mean}', period_stddev='{self.period_stddev}', period_max='{self.period_max}', stamp_age_mean='{self.stamp_age_mean}', stamp_age_stddev='{self.stamp_age_stddev}', stamp_age_max='{self.stamp_age_max}')>"""


Index('idx_topic_statistics_robot_topic', TopicStatistics.robot, TopicStatistics.topic)


class Odom(Base):
    __tablename__ = 'ts_odom'
    __table_args__ = ({'timescaledb_hypertable': {'time_column_name': 'time', 'chunk_time_interval': '1 day'}})

    time = Column(DateTime(timezone=True), nullable=False, primary_key=True)
    msg_time = Column(DateTime(timezone=True), nullable=True)
    robot = Column(String, nullable=False)
    frame = Column(String, nullable=False)
    position = Column(ARRAY(DOUBLE_PRECISION), nullable=False)
    orientation = Column(ARRAY(DOUBLE_PRECISION), nullable=False)
    rpy = Column(ARRAY(DOUBLE_PRECISION), nullable=False)

    def __repr__(self):
        return f"""<Odom(time='{self.time}', msg_time='{self.msg_time}', robot='{self.robot}', frame='{self.frame}', position='{self.position}', orientation='{self.orientation}', rpy='{self.rpy}')>"""


Index('idx_odom_robot_frame', Odom.robot, Odom.frame)



if __name__ == '__main__':
    from sqlalchemy import create_engine
    engine = create_engine('timescaledb://postgres:postgres@localhost:5432/postgres')
    Base.metadata.create_all(engine)

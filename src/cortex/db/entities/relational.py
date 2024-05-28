# Path: cortex/db/entities/relational.py
# This file is automatically generated by cortex/db/generator

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

from sqlalchemy import Column, Integer, String, DateTime, Index, ARRAY, ForeignKey, DOUBLE_PRECISION, Boolean
from cortex.db.entities import Base


class Robot(Base):
    __tablename__ = 'rel_robot'
    __table_args__ = ({})
    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, unique=True, nullable=False)
    description = Column(String, nullable=False)

    def __repr__(self):
        return f"""<Robot(id='{self.id}', name='{self.name}', description='{self.description}')>"""


Index('idx_robot_name', Robot.name)


class Actuator(Base):
    __tablename__ = 'rel_actuator'
    __table_args__ = ({})
    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, unique=True, nullable=False)
    type = Column(String, nullable=False)
    make = Column(String, nullable=False)
    model = Column(String, nullable=False)
    serial_no = Column(String, nullable=True, )

    def __repr__(self):
        return f"""<Actuator(id='{self.id}', name='{self.name}', type='{self.type}', make='{self.make}', model='{self.model}', serial_no='{self.serial_no}')>"""


Index('idx_actuator_name', Actuator.name)


class Host(Base):
    __tablename__ = 'rel_host'
    __table_args__ = ({})
    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, unique=True, nullable=False)
    ip_address = Column(String, nullable=False)

    def __repr__(self):
        return f"""<Host(id='{self.id}', name='{self.name}', ip_address='{self.ip_address}')>"""


Index('idx_host_name', Host.name)


if __name__ == '__main__':
    from sqlalchemy import create_engine
    engine = create_engine('timescaledb+psycopg2://postgres:postgres@localhost:5432/postgres')
    Base.metadata.create_all(engine)

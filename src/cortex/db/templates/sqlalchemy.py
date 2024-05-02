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


SQLALCHEMY_IMPORT_TEMPLATE = """from sqlalchemy import Column, Integer, String, DateTime, Index, ARRAY, ForeignKey, DOUBLE_PRECISION
from cortex.db.entities import Base
"""

SQLALCHEMY_TEMPLATE = """class CLASS_NAME(Base):
    __tablename__ = TABLE_NAME
    __table_args__ = ({TABLE_ARGS})
    COLUMNS

    def __repr__(self):
        return REPRESENTATION
INDICES"""

TEST_SCRIPT = """
if __name__ == '__main__':
    from sqlalchemy import create_engine
    engine = create_engine('timescaledb+psycopg2://postgres:postgres@localhost:5432/postgres')
    Base.metadata.create_all(engine)
"""
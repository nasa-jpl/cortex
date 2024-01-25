from .base import Base
from .relational import *
from .hypertable import *

table_classes = [cls for cls in globals().values() if hasattr(cls, "__tablename__")]

# Create an object that uses the table name as the key
tables = {cls.__tablename__: cls for cls in table_classes}

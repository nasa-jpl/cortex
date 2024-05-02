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

import datetime
import enum
from cortex.db import TemporalCRTX
from cortex.db.entities import Annotation


class AnnotationLevel(enum.Enum):
    """Enumeration of the different levels of annotations."""

    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class AnnotationTags(enum.Enum):
    """Enumeration of the different tags that can be used for annotations."""

    USER = "USER"
    SYSTEM = "SYSTEM"
    STATE = "STATE"
    EVENT = "EVENT"
    TEST = "TEST"
    DEBUG = "DEBUG"
    PLAN = "PLAN"
    HUMAN_COMMAND = "HUMAN_COMMAND"
    AUTONOMOUS_COMMAND = "AUTONOMOUS_COMMAND"
    SENSOR = "SENSOR"
    RUN = "RUN"


class CRTXAnnotator:
    """CORTEX agent for adding annotations to the database."""

    def __init__(self, host, robot):
        self.host = host
        self.robot = robot
        self.db = TemporalCRTX()

    def annotation(self, msg):
        a = Annotation(
            time=datetime.datetime.fromtimestamp(msg.start_time.to_sec()),
            msg_time=datetime.datetime.now(),
            end_time=datetime.datetime.fromtimestamp(msg.end_time.to_sec()),
            robot=self.robot,
            host=self.host,
            label=msg.label,
            message=msg.message,
            tags=msg.tags.value,
            level=msg.level.value,
        )
        self.db.insert(a)

    def annotate(
        self,
        start_time: datetime,
        msg_time: datetime,
        end_time: datetime,
        label: str,
        message: str,
        tags: list = None,
        level: AnnotationLevel = AnnotationLevel.INFO,
    ):
        """Annotate data with a label and message using now() as the time. Optionally, add tags and a level."""
        annotation = Annotation(
            time=start_time,
            msg_time=msg_time,
            end_time=end_time,
            robot=self.robot,
            host=self.host,
            label=label,
            message=message,
            tags=tags,
            level=level.value,
        )
        return annotation

    def now_annotation(
        self,
        label: str,
        message: str,
        tags: list = None,
        level: AnnotationLevel = AnnotationLevel.INFO,
    ):
        """Annotate data with a label and message using now() as the time. Optionally, add tags and a level."""
        label_time = datetime.datetime.now().isoformat()
        annotation = Annotation(
            time=label_time,
            msg_time=label_time,
            label=label,
            message=message,
            tags=tags,
            level=level.value,
        )
        return annotation

    def ranged_annotation(
        self,
        start_time: datetime,
        msg_time: datetime,
        end_time: datetime,
        label: str,
        message: str,
        tags: list = None,
        level: AnnotationLevel = AnnotationLevel.INFO,
    ):
        """Annotate data with a label and message using now() as the time. Optionally, add tags and a level."""
        annotation = Annotation(
            time=start_time,
            msg_time=msg_time,
            label=label,
            message=message,
            tags=tags,
            level=level.value,
            end_time=end_time,
        )
        return annotation

    def info(self, label: str, message: str, tags: list = None):
        """Annotate data with an INFO level."""
        return self.now_annotation(label, message, tags, AnnotationLevel.INFO)

    def warning(self, label: str, message: str, tags: list = None):
        """Annotate data with a WARNING level."""
        return self.now_annotation(label, message, tags, AnnotationLevel.WARNING)

    def error(self, label: str, message: str, tags: list = None):
        """Annotate data with an ERROR level."""
        return self.now_annotation(label, message, tags, AnnotationLevel.ERROR)

    def critical(self, label: str, message: str, tags: list = None):
        """Annotate data with a CRITICAL level."""
        return self.now_annotation(label, message, tags, AnnotationLevel.CRITICAL)

    def __del__(self):
        if getattr(self, "__db", None):
            self.db.shutdown(block=True)

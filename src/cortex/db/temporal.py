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

import threading
import queue
import time
import sys
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.orm.session import close_all_sessions
from cortex.db.entities import *


class TemporalCRTX:
    """A library for interfacing with the CORTEX database."""

    def __init__(
        self,
        hostname="127.0.0.1",
        port=5432,
        batch_size=2500,
        batch_timeout=2,
        migrate: bool = False,
        database="postgres",
        logger=None,
    ):
        """This class is a wrapper around SQLAlchemy and psycopg2, and provides a
        thread-safe interface for inserting data into the database."""
        self.__hostname = hostname
        self.__port = port
        self.__database = database
        self.__logger = logger
        self.__url = (
            f"timescaledb+psycopg2://postgres:postgres@{hostname}:{port}/{database}"
        )

        self.__engine = create_engine(self.__url, echo=False)
        self.__session = sessionmaker(bind=self.__engine)
        Base.metadata.create_all(self.__engine)

        self.batch_size: int = batch_size
        self.batch_timeout = batch_timeout
        self.__conditional_lock = threading.Condition()
        self.__killed = False
        self.__queue = queue.Queue()
        self.__queue_size = 0
        self.__previous_time = time.time()
        self.__worker_thread = threading.Thread(target=self.__worker, daemon=False)
        self.__worker_thread.start()

    def get_session(self):
        session = self.__session()
        return session

    def insert(self, data):
        if not isinstance(data, list):
            data = [data]
        with self.__conditional_lock:
            for entity in data:
                try:
                    self.__queue.put(entity)
                    self.__queue_size += 1
                except queue.Full:
                    print(
                        f"EELSdB: Failed to insert entity due to full queue (size={self.__queue_size})."
                    )
                    continue
            if self.__queue_size >= self.batch_size:
                self.__conditional_lock.notify()

    def __log_or_print(self, message, log_throttle_time=None):
        """A helper function that logs a message if ROS is running, otherwise prints it to stdout."""
        print(f"Logging: {message}")

        # If a logger is provided, use it
        if self.__logger:
            self.__logger.info(message)
            return

        # Otherwise, try to use ROS
        try:
            import rospy

            if not rospy.is_shutdown():
                if log_throttle_time:
                    rospy.loginfo_throttle(log_throttle_time, message)
                else:
                    rospy.loginfo(message)
            else:
                print(message)
        except:
            print(message)

    def __should_work(self):
        """A boolean function that returns True if the batch size or timeout has been reached."""
        return self.__queue_size >= self.batch_size

    def __worker(self):
        """Worker thread, periodically checks for new data in the queue and inserts it into the database in batches."""
        with self.get_session() as session:
            while not self.__killed:
                with self.__conditional_lock:
                    self.__conditional_lock.wait_for(
                        self.__should_work, self.batch_timeout
                    )
                    batch = []
                    while self.__queue_size > 0:
                        entity = self.__queue.get()
                        self.__queue_size -= 1
                        self.__queue.task_done()
                        batch.append(entity)

                        self.__log_or_print(
                            f"EELSdB: Got batch of {len(batch)} entities."
                        )
                    if len(batch) > 0:
                        try:
                            session.add_all(batch)
                            session.commit()
                            self.__previous_time = time.time()
                        except Exception as e:
                            self.__log_or_print(
                                f"[ERROR] EELSdB: Encountered SQLAlchemy error while batch inserting data: {e}"
                            )
                            session.rollback()
            with self.__conditional_lock:
                batch = []
                while self.__queue_size > 0:
                    entity = self.__queue.get()
                    self.__queue_size -= 1
                    self.__queue.task_done()
                    batch.append(entity)
                if len(batch) > 0:
                    session.add_all(batch)
                    session.commit()
        self.__log_or_print(f"[WARN] EELSdB: Worker thread exiting...")

    def shutdown(self, block=False, timeout=30.0):
        # Stop the worker thread, ensure queue is empty
        self.__log_or_print(f"EELSdB: Shutting down...")
        self.__killed = True

        self.__log_or_print(f"EELSdB: Getting CV lock...")
        with self.__conditional_lock:
            self.__log_or_print(f"EELSdB: Notifying worker thread...")
            self.__conditional_lock.notify_all()

        self.__log_or_print(f"EELSdB: Joining Queue...")
        self.__queue.join()

        # Close the EELSdB Session
        self.__log_or_print(f"EELSdB: Closing session...")
        close_all_sessions()

        # Close the connection to the database
        self.__log_or_print(f"EELSdB: Disposing engine...")
        self.__engine.dispose()

        if not block:
            sys.exit()


if __name__ == "__main__":
    db = TemporalCRTX()
    db.shutdown()

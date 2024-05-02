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
import psycopg2
import queue
import time
import sys
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.orm.session import close_all_sessions
from cortex.db.entities import *


class TemporalCRTX:
    """A library for interfacing with the CORTEX database.

    This class is a wrapper around SQLAlchemy and psycopg2, and provides a
        thread-safe interface for inserting data into the database.

    Args:
        hostname (str): The hostname used to connect to the database.
        port (int): The port used to connect to the database.
        batch_size (int): The number of entities to batch insert into the database.
        batch_timeout (float): The time in seconds to wait before inserting a batch of entities, even if the batch size has not been reached.
        database (str): The name of the database.
        logger (Logger): A logger object for logging messages.
        logging (bool): A boolean flag to enable or disable logging.
    """

    def __init__(
        self,
        hostname="127.0.0.1",
        port=5432,
        batch_size=2500,
        batch_timeout=2.0,
        database="postgres",
        logger=None,
        logging=False,
    ):
        """"""
        self.__killed = False
        self.__hostname = hostname
        self.__port = port
        self.__database = database
        self.__logger = logger
        self.__logging = logging
        self.__url = (
            f"timescaledb+psycopg2://postgres:postgres@{hostname}:{port}/{database}"
        )
        self.batch_size: int = batch_size
        self.batch_timeout = batch_timeout
        self.__next_update_timeout = 0
        self.__conditional_lock = threading.Condition()
        self.__queue = queue.Queue()
        self.__queue_size = 0
        self.__previous_time = time.time()
        self.__worker_thread = threading.Thread(target=self.__worker, daemon=True)

        self.__setup()
        self.__worker_thread.start()
        self.__log_or_print(f"Initialized {self.__repr__()}")

    def __repr__(self):
        return f"TemporalCRTX(hostname={self.__hostname}, port={self.__port}, batch_size={self.batch_size}, batch_timeout={self.batch_timeout}, database={self.__database})"

    def __setup(self):
        try:
            self.__engine = create_engine(self.__url, echo=False)
        except psycopg2.OperationalError as e:
            print(
                f"TemporalCRTX: Failed to create engine. Make sure the database is running and accessible.\n{e}"
            )
            sys.exit(1)

        try:
            self.__session = sessionmaker(bind=self.__engine)
        except Exception as e:
            print(
                f"TemporalCRTX: Failed to create session. Make sure the database is running and accessible.\n{e}"
            )
            sys.exit(1)

        try:
            Base.metadata.create_all(self.__engine)
        except Exception as e:
            print(
                f"Failed to create tables. Make sure the database is running and accessible.\n{e}"
            )
            sys.exit(1)

    def get_session(self):
        """Get a new session object for interacting with the database using SQLAlchemy."""
        session = self.__session()
        return session

    def delete(self, query, _filter):
        """Delete data from the database matching the given query and filter.

        :param query: The entity (table) to delete from (e.g. 'DataProduct')
        :param _filter: The filter to apply to the query (e.g. 'DataProduct.id == 1')
        """
        with self.get_session() as session:
            data = session.query(query).filter(_filter)
            data.delete()
            session.commit()

    def insert(self, data):
        """Insert data into the database.

        Args:
            data: The data to insert into the database. This can be a single entity or a list of entities.
        """
        if not isinstance(data, list):
            data = [data]
        with self.__conditional_lock:
            for entity in data:
                try:
                    self.__queue.put(entity)
                    self.__queue_size += 1
                except queue.Full:
                    print(
                        f"TemporalCRTX: Failed to insert entity due to full queue (size={self.__queue_size})."
                    )
                    continue
            if self.__queue_size >= self.batch_size:
                self.__conditional_lock.notify()

    def query(self, _query, _filter):
        """A convenience method for querying the database."""
        with self.get_session() as session:
            data = session.query(_query).filter(_filter).all()
            return data

    def __log_or_print(self, message, log_throttle_time=None):
        """A helper function that logs a message if ROS is running, otherwise prints it to stdout.

        :param message: The message to log or print.
        :param log_throttle_time: The time in seconds to throttle the log message.
        """
        if not self.__logging:
            return

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

        # Use the same session throughout the worker thread lifetime
        with self.get_session() as session:

            # Main loop, run until killed
            while not self.__killed:
                start_time = time.time()

                with self.__conditional_lock:
                    # Wait for the batch size or timeout to be reached
                    self.__conditional_lock.wait_for(
                        self.__should_work, self.__next_update_timeout
                    )

                    # Next update timeout should be however long it takes to reach the batch timeout
                    self.__next_update_timeout = self.batch_timeout - (
                        time.time() - start_time
                    )
                    self.__next_update_timeout = max(0.0, self.__next_update_timeout)

                    batch = []

                    while self.__queue_size > 0:
                        entity = self.__queue.get()
                        self.__queue_size -= 1
                        self.__queue.task_done()
                        batch.append(entity)

                    if len(batch) > 0:
                        self.__log_or_print(
                            f"TemporalCRTX: Got batch of {len(batch)} entities.",
                        )
                        try:
                            session.add_all(batch)
                            session.commit()
                            self.__previous_time = time.time()
                        except Exception as e:
                            self.__log_or_print(
                                f"[ERROR] TemporalCRTX: Encountered SQLAlchemy error while batch inserting data: {e}"
                            )
                            session.rollback()

            # Insert any remaining data in the queue before exiting
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

        self.__log_or_print(f"[WARN] TemporalCRTX: Worker thread exiting...")

    def shutdown(self, block=True, timeout=30.0):
        # Stop the worker thread, ensure queue is empty
        self.__log_or_print(f"TemporalCRTX: Shutting down...")
        self.__killed = True

        self.__log_or_print(f"TemporalCRTX: Getting CV lock...")
        with self.__conditional_lock:
            self.__log_or_print(f"TemporalCRTX: Notifying worker thread...")
            self.__conditional_lock.notify_all()

        self.__log_or_print(f"TemporalCRTX: Joining Queue...")
        self.__queue.join()

        # Close the TemporalCRTX Session
        self.__log_or_print(f"TemporalCRTX: Closing session...")

        try:
            close_all_sessions()
        except Exception as e:
            self.__log_or_print(f"TemporalCRTX: Failed to close all sessions: {e}")

        # Close the connection to the database
        self.__log_or_print(f"TemporalCRTX: Disposing engine...")
        self.__engine.dispose()

        if not block:
            # Wait for timeout seconds then exit
            time.sleep(timeout)
            sys.exit()

    def __del__(self):
        if getattr(self, "__killed", False):
            return
        else:
            self.shutdown(block=True)

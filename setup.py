#!/usr/bin/env python3

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

import os
import subprocess
from setuptools import setup, find_packages, Command
import pathlib

here = pathlib.Path(__file__).parent.resolve()
long_description = (here / "README.md").read_text(encoding="utf-8")
cortex_packages = find_packages(where="src")


class GenerateDBTables(Command):
    """Only required when the tables have been modified and need to be regenerated."""
    description = "(Re)Generate database tables."
    user_options = []

    def initialize_options(self):
        self.gen_path = "src/cortex/db/generator.py"
        self.cfg_path = "src/cortex/config/database/"
        self.output_path = "src/cortex/db/entities/"

    def finalize_options(self):
        pass

    def run(self):
        # Get user input to make sure they want to regenerate the tables
        print("\nThis command will regenerate the database tables. All generated files will be overwritten.")
        print("Are you sure you want to continue? (yes/no)")
        response = input("> ")
        if response == "yes":
            try:
                res = subprocess.run([
                    "python3", self.gen_path,
                    f"--config={self.cfg_path}",
                    f"--output={self.output_path}"
                ])
                res.check_returncode()
            except subprocess.CalledProcessError as e:
                print(f"Failed to generate tables: {e}")
                return
            print("Tables generated!")
        else:
            print("Table generation aborted.")


class DockerCommands(Command):
    """Initialize the Docker environment."""
    description = "Initialize the Docker environment."
    user_options = [
        ("start", None, "Start the Docker environment."),
        ("stop", None, "Stop the Docker environment."),
        ("restart", None, "Restart the Docker environment."),
        ("purge", None, "Wipe the containers and volumes (use with caution)."),
    ]

    def initialize_options(self):

        try:
            import dotenv
        except:
            print("Please install the dotenv package to use this command.")
            return

        dotenv.load_dotenv(".env")

        self.start = False
        self.stop = False
        self.restart = False
        self.purge = False

    def finalize_options(self):
        pass

    def run(self):
        if self.start:
            subprocess.run(["docker", "compose", "up", "-d"])
        elif self.stop:
            subprocess.run(["docker", "compose", "down"])
        elif self.restart:
            subprocess.run(["docker", "compose", "restart"])
        elif self.purge:
            subprocess.run(["docker", "compose", "down", "-v", "--remove-orphans"])
        else:
            print("\nPlease specify an option (start, stop, restart, purge).")
            print("Example: python3 setup docker --start")
            return


class DatabaseCommands(Command):
    """Initialize the database."""
    description = "Commands for controlling the Postgres database."
    user_options = [
        ("init", None, "Initialize the database. Assumes the database is already running."),
        ("wipe", None, "Wipe the database (use with caution)."),
    ]

    def initialize_options(self):
        self.init = False
        self.wipe = False

    def finalize_options(self):
        pass

    def run(self):
        if self.init:
            # Import the entities from cortex.db.entities and initialize the database
            try:
                import dotenv
                from sqlalchemy import create_engine
                from cortex.db.entities import Base
            except:
                print(f"Please run this script using the install command before initializing the database.")
                print(f"Example: python3 setup.py install")
                return

            dotenv.load_dotenv(".env")
            HOSTNAME = os.getenv("DB_HOSTNAME")
            PORT = os.getenv("DB_PORT")
            USERNAME = os.getenv("DB_USER")
            PASSWORD = os.getenv("DB_PASSWORD")
            DATABASE = os.getenv("DB_NAME")

            DB_URL = f"postgresql+psycopg2://{USERNAME}:{PASSWORD}@{HOSTNAME}:{PORT}/{DATABASE}"
            engine = create_engine(DB_URL)

            print(f"Initializing... {engine}")

            Base.metadata.create_all(engine)
        elif self.wipe:
            try:
                import dotenv
            except ModuleNotFoundError:
                print("Please install the dotenv package to use this command.")
                print("Example: pip install python-dotenv")
                return

            dotenv.load_dotenv(".env")
            persistence_directory = os.getenv("PERSISTENCE_DIRECTORY")
            db_directory = f"{persistence_directory}/cortex/timescaledb"
            db_directory = os.path.expanduser(db_directory)

            # Make sure it exists and get user confirmation
            if os.path.exists(db_directory):
                # Warn the user and ask if they want to continue. Use red text for emphasis.
                print("\n\033[91mWARNING: This will delete all data in the database.\033[0m")
                print(f"Are you sure you want to wipe the database at {db_directory}? (yes/no)")
                response = input("> ")

                if response == "yes":
                    subprocess.run(["docker", "compose", "down", "-v", "--remove-orphans"])

                    # Run the rm -rf command as root or using sudo
                    subprocess.run(["sudo", "rm", "-rf", db_directory])
                    print("Database wiped!")
                else:
                    print("Database wipe aborted.")
            else:
                print(f"Database directory not found at {db_directory}.")

        else:
            print("\nPlease specify an option (init, wipe).")
            print("Example: python3 setup database --init")
            return


setup(
    name="jpl-cortex",
    cmdclass={
        "tables": GenerateDBTables,
        "docker": DockerCommands,
        "database": DatabaseCommands,
    },
    version="1.0.0",
    license="Apache 2.0",
    platforms="Ubuntu 20.04",
    description="A framework for accelerating robotics development through a combination of modern data infrastructure, test automation, and intelligent data analysis.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/nasa-jpl/cortex",
    author="Rob Royce",
    author_email="Rob.Royce@jpl.nasa.gov",
    classifiers=[
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Developers :: Data Scientists :: Robotics Engineers",
        "Topic :: Software Development :: Build Tools :: Data Science :: Machine Learning :: Robotics",
        "License :: OSI Approved :: Apache 2.0 License",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
    keywords="Robotics, Data Science, Machine Learning, Data Engineering, Data Infrastructure, Data Analysis",
    package_dir={"": "src"},
    packages=cortex_packages,
    python_requires=">=3.8, <4",
    install_requires=[
        "PyYAML==6.0.1",
        "psutil==5.9.7",
        "psycopg2==2.9.9",
        "SQLAlchemy==2.0.25",
        "GeoAlchemy2==0.14.3",
        "sqlalchemy-timescaledb==0.4.1",
        "python-dotenv>=1.0.1",
    ],
    project_urls={  # Optional
        "Bug Reports": "https://github.com/nasa-jpl/cortex/issues",
        "Source": "https://github.com/nasa-jpl/cortex",
    },
)

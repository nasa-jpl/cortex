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

from setuptools import setup, find_packages
import pathlib

here = pathlib.Path(__file__).parent.resolve()
long_description = (here / "README.md").read_text(encoding="utf-8")
cortex_packages = find_packages(where="src")
print(f"Cortex packages: ", cortex_packages)

setup(
    name="jpl-neo-cortex",
    version="1.0.0",
    license="Apache 2.0",
    platforms="Ubuntu 20.04",
    description="A framework for accelerating robotics development through a combination of modern data infrastructure, test automation, and intelligent data analysis.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/nasa-jpl/neo-cortex",
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
    ],
    project_urls={  # Optional
        "Bug Reports": "https://github.com/nasa-jpl/neo-cortex/issues",
        "Source": "https://github.com/nasa-jpl/neo-cortex",
    },
)

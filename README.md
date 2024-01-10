# CORTEX: Continuous Optimization in Robotics via Test and Experimentation
The CORTEX project aims to develop a framework for accelerated robotics development using
modern artificial intelligence techniques. It also borrows heavily from the "Big Data" systems
employed by companies such as Google, Facebook, and Amazon. The framework enables developers to
rapidly prototype and test new algorithms and ideas in a simulated environment, and then
deploy them to real robots with minimal effort.

CORTEX was developed as part of the NEO Autonomy System at NASA Jet Propulsion Laboratory (hence, NEO-CORTEX).

## Configuration
The following components of CORTEX can be configured:

### Database (PostgreSQL w/ TimescaleDB)
See [config/timescaledb/README.md](config/timescaledb/README.md)

### Device Metrics (Telegraf)
See [config/telegraf/README.md](config/telegraf/README.md)

### Grafana (Dashboard)
See [config/grafana/README.md](config/grafana/README.md)

### ROS Bags
See [config/rosbags/README.md](config/rosbags/README.md)

### ROS Workers
See [config/workers/README.md](config/workers/README.md)

## Architecture
CORTEX is intended to work with a wide variety of robots and devices. It is designed to be as
modular as possible, so that it can be easily adapted to work with different devices and
environments. The following diagram shows the high-level architecture of CORTEX:
[![CORTEX Architecture](docs/diagrams/cortex_architecture.png)](docs/diagrams/cortex_architecture.png)

#### Agents
CORTEX Agents can be thought of as components that are responsible for performing specific tasks in a robotics system.
They are typically implemented as ROS nodes, and are configured using YAML files (where applicable).

CORTEX Provides the following Agents:

- **orchestrator**: The orchestrator node is responsible for managing the entire CORTEX system.
  It is responsible for starting and stopping the various components of the system, and for
  coordinating the communication between them.
- **monitor**: The monitor node is responsible for collecting resource utilization metrics (CPU/Memory) from
  ROS nodes running on the system.
- **worker**: The worker node is responsible for listening to ROS topics and collecting data
  from them. It is also responsible for sending data to the database. Note that the worker node
  will typically subsample the data before sending it to the database, in order to reduce the
  amount of data that is sent.
- **sampler**: The sampler node is responsible for collecting data from sources that do not publish
  ROS topics. This includes collecting data by performing service/action calls, or by reading
  data from files.
- **bagger**: The bagger node is responsible for recording ROS topics to a bag file. This is
  useful for debugging and for replaying experiments. Note that the bagger node will typically
  record data at its full rate, and is therefore larger than the data that is sent to the database.
- **annotator**: The annotator node is responsible for recording events that occur during an
  experiment. This includes recording the start and end times of an experiment, as well as
  significant events such as state transitions, reaching a goal, crashing, or encountering an obstacle.

### Libraries
CORTEX is built around a set of Python libraries that provide the core functionality of the system. Future
work will include migrating these libraries to C++ for improved performance.

The following describes the CORTEX Python library in terms of its modules. This list does not include the ROS nodes that
are provided by CORTEX. See the [ROS Nodes](#ros-nodes).

- **TemporaLobe**: A library for interfacing with the CORTEX database.
- **CerebraLobe**: A library for automating test experiments and performing monte-carlo simulations.
- **OccipitaLobe**: A library for visualizing data collected by CORTEX.

### Services

## Implementation
The following sections describe the various implementations of CORTEX (current and future).

### ROS1
Though CORTEX is designed to be as modular as possible, it is currently built around the
[Robot Operating System (ROS)](https://www.ros.org/). ROS is a set of software libraries and tools
that help developers build robot applications. It provides hardware abstraction, device drivers,
libraries, visualizers, message-passing, package management, and more. ROS is licensed under an
open source, BSD license.

### ROS2
As it becomes more mature, we will begin to migrate the CORTEX framework to work with ROS2. This will
allow us to take advantage of the new features and improvements that ROS2 offers, such as better
real-time performance, improved security, and better support for embedded systems.

## Getting Started
[//]: # (TODO)

### Prerequisites
[//]: # (TODO)

### Installation
[//]: # (TODO)

### Docker
[//]: # (TODO)

# References
[//]: # (TODO)
- NEO Autonomy System

# Acknowledgements
[//]: # (TODO: JPL Team)
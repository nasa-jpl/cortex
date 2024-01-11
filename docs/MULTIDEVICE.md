# CORTEX Mutli-device Configuration

This document describes how to configure CORTEX to work with multiple devices (hosts).
Using this approach, you can run the CORTEX services on multiple devices while allowing them
to send data to the same database. This is useful for cases in which more than one computer
is used for testing, such as when using a desktop computer for teleoperation and an on-board
computer for autonomy.

## Overview

The following diagram shows the basic architecture of a multi-device CORTEX system:
[![CORTEX Multi-device Architecture](docs/diagrams/cortex_multi_device_architecture.png)](docs/diagrams/cortex_multi_device_architecture.png)

In this configuration, the `orchestrator` node is run on the `main` device. The `orchestrator` is responsible for
starting and stopping various components and services of the CORTEX system, and for configuring the environment
variables used by the other devices. By doing so, the global configuration of the system can be
specified and managed from a single device, rather than having to configure each device individually.

The `orchestrator` is also responsible for verifying that the database is available before starting any of the other
nodes and services. This is important because the other nodes will attempt to connect to the database when they are
started, and will fail if the database is not available.

The database and dashboard may be hosted on a device of your choosing, so long as all devices have access to it and
specify the correct hostname in their configuration. For best performance, it is recommended to run the database and
dashboard on a dedicated device that is separate from any device that requires high performance and low latency (e.g.
controls, autonomy, etc.)

## Configuration

In a multi-device configuration, each device will run a subset of the CORTEX nodes.
Which nodes are run on which devices can be determined by using one of the following
methods:

- By setting the `DEVICE_ROLE` environment variable on each device
- By assigning nodes to specific `HOSTNAME`'s

### Setting the `DEVICE_ROLE` Environment Variable

The more generalized approach to configuring a multi-device system is to assign a `DEVICE_ROLE` to each of the devices.
With a few notable exceptions (see below), each role can be used on as many devices as you want. For example, if you
have three devices, two of them can be assigned the `sensing` role, and the third can be assigned the `control` role.
Additionally, you can use the role of a particular device as a condition for more complex configuration. For example,
you may want the UI components to start automatically on the `base` device, but not on any other device (while still
making them available for manual startup).

The following roles are supported:

- `main`: The main device is responsible for running the `orchestrator` node (see above).
- `base`: The base (or base station) is primarily used for teleoperation and other UI-related tasks. It is also a
  reasonable choice for running the database and dashboard when a dedicated device is not available.
- `control`: The control device is responsible for running the control system and is frequently used for
  directly interfacing with hardware. Given its critical role and real-time requirements, we recommend
  only running the `monitor` and `worker` nodes on this device, and restricting the worker to topics
  that are directly related to the control system.
- `sensing`: The sensing device is usually responsible for running the perception system in isolation, or for
  feeding sensor data into the wider system over the network.
- `context`: A context device is typically responsible for providing context during an experiment. This includes
  things like recording video of the robot in its environment, taking notes, or annotating events that occur during an
  experiment.

#### Exceptions

Since it is not always desirable to have a single device acting as an orchestrator, the role of `main` can also be
assigned to a device using any of the other roles. This "dual-role" device can be specified by setting the `DEVICE_ROLE`
environment variable in the following format: `<device_role>:main`. For example, if you want to run the `main` and`base`
roles on the same device, you would set the `DEVICE_ROLE` environment variable to `base:main`. This method **only**
works for the `main` role. Additionally, **There can only be one main device, and this device should always be
launched first to ensure the orchestrator is available when other devices connect to the system.**

## Nuances and Best Practices

### Monitor Node

When running a multi-device configuration, all devices will run the `monitor` node,
since resource utilization can only be determined on the host device for a
particular set of nodes. The `monitor` is already configured for this type of operation
and therefore requires no additional setup.

### Worker Node

The `worker` node, on the other hand, requires careful planning and deliberation
in order to ensure optimal performance. Specifically, the `worker` node should be
run on the same device as the ROS node that is publishing the data being recorded.
This will ensure that the data does not need to be sent over the network when
it otherwise would not need to be. For example, if you are recording data from
a LIDAR sensor, you should run the `worker` node on the same device as the LIDAR
sensor. Attempting to run the `worker` node on a different device will result in
a very large amount of data being sent over the network, which will likely cause
congestion and poor performance. Of course, there are exceptions to this rule,
but those exceptions should be carefully considered and tested before being
deployed in a production environment.

### Sensing Role

The `sensing` device is likely to be constrained in terms of resource availability, especially CPU, memory, and network
bandwidth. For this reason, the `sensing` device is not a good candidate for running anything other than the `monitor`
and `bagger` nodes. Additionally, if a `bagger` node is run on this device, we recommend setting its trigger level
to `high`, so that it only records data when the `Log Extra` button is enabled.

# Additional Notes
In future versions of CORTEX, we plan to facilitate a more sophisticated method
of deploying nodes (and therefore workers) across multiple devices. While the implementation
details are still being worked out, the basic idea is to optimize the distribution
of nodes across devices based on the network topology, actuator/sensor locality, and
resource utilization, and other factors.

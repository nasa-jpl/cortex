# CORTEX Environment Configuration

CORTEX allows users to configure the environment using two types of environment variables:

- **system environment variables**: Used to configure the database and other system-wide settings.
- **device environment variables**: Used to configure device-specific settings, like `HOSTNAME` and `DEVICE_ROLE`.

These environment variables are used throughout CORTEX for various purposes, including:

* Establishing connections to the database
* Configuring the behavior of nodes
* Assigning nodes to specific devices in a multi-device configuration
* And more

## System-wide Configuration

CORTEX aims to make system-wide configuration easy, especially in cases where multiple devices require the same
configured values. For instance, if all connected devices need to connect to the same database, it is easier to
configure the database hostname and port in a single place, rather than on each device. This is where system-wide
environment variables come in. These environment variables are used to configure the system as a whole, and are
typically set on the `main` device. In single-device configurations, these environment variables are set on the
device itself. In multi-device configurations, these environment variables are set on the `main` device, and are
propagated to the other devices using the `orchestrator` agent.

# List of Environment Variables

## System Environment

- `DB_HOSTNAME`: The hostname of the database server.
- `DB_PORT`: The port of the database server.

## Device Environment

- `HOSTNAME`: The hostname of the device. This is used to assign nodes to specific devices in a multi-device
  configuration. See [the multi-device documentation](./docs/MULTIDEVICE.md) for more information.

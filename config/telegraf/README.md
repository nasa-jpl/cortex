# About Telegraf

[Telegraf](https://www.influxdata.com/time-series-platform/telegraf/) is a plugin-driven server agent for collecting and
reporting metrics for all kinds of data from databases, systems, and IoT devices. In CORTEX, Telegraf is used to collect
metrics from all connected devices and (optionally) to ingest ROS logs, and send them to the TimescaleDB database.

While CORTEX implements its own data collection system, Telegraf is a well-established and
well-maintained tool that is used by many companies and organizations specifically for collecting **device metrics**.
It is also very flexible and can be easily configured to collect data from a wide variety of sources.
For these reasons, we have decided to use Telegraf for part of our data collection and ingestion system in CORTEX.

# Telegraf Configuration

This directory contains configuration files for the various Telegraf plugins.

- `default.conf`: contains standard plugins for collecting device metrics, such as CPU, memory, disk, and network.
- `roslogs.conf`: contains a plugin for parsing and collecting ROS logs.

**Note:** The `default.conf` file is required for Telegraf to run. The `roslogs.conf` file is optional.
The reason why `roslogs` are not included in `default.conf` is because, unless your system is well
kept and logging is under control, the `roslogs` plugin can be very resource intensive, to the
point that it can cause the database to become unresponsive. For this reason, it is recommended
to only enable the `roslogs` plugin when you need to collect logs for debugging purposes.

# Getting Started (Manually)

- Install [Telegraf](https://www.influxdata.com/time-series-platform/telegraf/)
- Run `telegraf -config <path to config file>`
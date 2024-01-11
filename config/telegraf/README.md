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
- `roslogs.conf`: contains a plugin for parsing and 'grokking' ROS logs (see below).

**Note:** The `default.conf` file is required for Telegraf to run. The `roslogs.conf` file is optional.
The reason why `roslogs` are not included in `default.conf` is because, unless your system is well
kept and logging is under control, the `roslogs` plugin can be very resource intensive, to the
point that it can cause the database to become unresponsive. For this reason, it is recommended
to only enable the `roslogs` plugin when you need to collect logs for debugging purposes.

# Getting Started

- Install [Telegraf](https://www.influxdata.com/time-series-platform/telegraf/)

## Ingesting Device Metrics

Users have the choice of running Telegraf consistently in the background, or running it only when they need to collect
metrics for debugging purposes. If you choose to run Telegraf consistently in the background, you will probably
want to run it as a system service. Just keep in mind that you will be accumulating data even when not actively running
experiments. On the other hand, if you choose to run Telegraf only when you need to collect metrics, you will need to
ensure it is triggered at the start of each run. CORTEX provides facilities for doing this automatically, but you will
need to ensure the appropriate configuration is in place, and that the services are launched appropriately.

### Running Telegraf as a System Service

[//]: # (TODO)

### Running Telegraf Manually

[//]: # (TODO)

### Running Telegraf Automatically

[//]: # (TODO)

## Ingesting ROS Logs

To ingest ROS logs, you will need to run Telegraf with the `roslogs.conf` configuration file. This file contains a
plugin for parsing and 'grokking' ROS logs. You must specify the directory where the logs are stored using the
`ROS_LOG_DIR` environment variable. For example, if your logs are stored in `/home/user/logs`, you would run Telegraf
with the following command:

```bash
ROS_LOG_DIR=/home/user/logs telegraf -config <full path to roslogs.conf>
```

You may choose to run this command at the start of a run and let it run in the background, or you may choose to run it
manually when you need to collect logs for debugging purposes. Since the `roslogs` plugin can be very resource intensive
and cause the database to become unresponsive, we recommend only running it when you need to collect logs for debugging.

# Grokking ROS Logs

> [Grok](https://www.elastic.co/guide/en/logstash/current/plugins-filters-grok.html) is a powerful tool for extracting
> structured data from unstructured text.

If you look in the `roslogs.conf` file, you will see that it contains a list of `grok` patterns. These patterns are used
to parse the ROS logs and extract data from them. A curated set of `grok` patterns are defined in
the`grok_patterns` section of the configuration file. These patterns cover most of the common ROS log messages, but you
may find that you need to add your own patterns to handle custom log messages. To do this, you can add your own patterns
to the list.

One method for doing so is to add your patterns to the [Grok Debugger](https://grokdebugger.com/), along with some
example log messages. Then, you can use the debugger to test your patterns and make sure they work as expected. Once you
are satisfied with your patterns, you can copy them into the `roslogs.conf` file.

Keep in mind that the patterns in `roslogs.conf` make extensive use of escape sequences, whereas the patterns in the
Grok Debugger do not require them. Therefore, you will need to be careful when copying patterns from the Grok Debugger,
and multiple test runs may be required to get the patterns right. It is also worth noting that using too many Grok
patterns can cause performance issues, because each pattern is applied to every log message. Therefore, it is
recommended to only use the patterns that you absolutely need.

Finally, one method for filtering out unwanted log messages is to use the `LOGLEVEL` custom pattern defined in the
`grok_custom_patterns` section of the configuration file. Since `grok` effectively ignores any patterns that are not
included in the expression, currently `(WARN|WARNING|ERROR|FATAL|UNKNOWN)`, it will ignore any `LOG`, `INFO`, or `DEBUG`
level messages. For instance, you can remove `WARN` to ignore messages that have a log level of `WARN`, or you can add
`INFO` if you want to include messages that have a log level of `INFO`.

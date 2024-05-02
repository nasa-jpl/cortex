# CORTEX - ROS1 Package

This package is a ROS1 implementation of the CORTEX data framework. It provides a set of ROS nodes that use CORTEX
agents to monitor and record data from a running ROS system. The annotator allows other nodes to record annotations.
This is especially useful for recording events that occur during an experiment, such as state transitions, reaching a
goal, crashing, or encountering an obstacle.

## Usage

```bash
# Launch all nodes
roslaunch cortex cortex.launch

# Launch the monitor node
roslaunch cortex monitor.launch
```

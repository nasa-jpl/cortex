# CORTEX Worker Configuration

Workers are split into different categories based on the complexity of their respective
pipelines:

# Basic Workers

Basic workers are the most common and least complex. They listen to a single topic and
transform the data either immediately (full rate), on change (limited rate), or at a specified rate.

## Specification

To specify a basic worker, add a new entry to `cortex/config/workers/basic.yaml`.

**Required Fields:**
- topic: the name of the topic to subscribe to (e.g. /joint_states)
- data_type: the data type, typically a ROS msg type (e.g. JointState)
- msg_pkg: the package that contains the data type (e.g. sensor_msgs.msg)
- target: the name of the database table to insert into (e.g. joint_states_actual) 

**Optional Fields:**
- preprocessors: a list of preprocessor functions (see cortex/db/transforms)
- transforms: a list of transform functions (see cortex/db/transforms)
- hz: a sample rate for the specified topic
- change_fields: a list of fields to check for value changes
- global_args: a list of arguments that will always be added to the database rows (e.g. robot name)

## Behavior

The rate at which messages are received and inserted into the database will depend on the
configuration, specifically on the `change_fields` and `hz` fields. The following
table describes the expected behavior depending on how those fields are set:

| `change_fields` | `hz` | behavior  |
|-----------------|------|-----------|
| None            | None | Full Rate | 
| Non-empty       | None | On Change |
| None            | >0   | Limited   |
| Non-empty       | >0   | On Change |

**Note that setting `change_fields` will effectively override the `hz` setting.**



# Not Yet Implemented (coming soon)

- `basic`: Transform messages as they are received (immediately) or at a specific rate (sampled)
![Basic Worker](../../../../docs/agents/worker/basic.png)


**Not Yet Implemented (coming soon)**

- `latest`: Wait for all topics to be ready before transforming, overwriting older messages until all topics are
  received.
![Latest Worker](../../../../docs/agents/worker/latest.png)

- `sequential`: Only process topics once they have been received **in the specified order**. 
If messages are received out of turn, all messages are discarded and the cycle starts over. 

![Sequential Worker](../../../../docs/agents/worker/sequential.png)


- `approximate`: Apply transforms only if a message has been received on all specified topics within N seconds of each
  other.

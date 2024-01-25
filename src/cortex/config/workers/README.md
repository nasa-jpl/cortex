# CORTEX Worker Configuration

## Topic Pipes

Valid join types for multiple topics:

- `independent`: Transform messages as they are received (immediately).
- `latest`: Wait for all topics to be ready before transforming, overwriting older messages until all topics are
  received.
- `approximate`: Apply transforms only if a message has been received on all specified topics within N seconds of each
  other.
- `exact`: Apply transforms only if a message has been received on all specified topics with the same exact timestamp.
- `sequential`:
- `sample` (default): Transform the latest messages from each topic at the specified rate, regardless of which messages
  are available.

### Sample

Transform the latest messages from each topic at the specified rate, regardless of which messages
are available.

### Sequential

Apply transforms only after messages are received in the exact order they are listed.
A new message on the first topic will always reset the sequence.

## Constraints

### Robot Models

Only instantiate a worker if the current `ROBOT_MODEL` is listed in the `robot_models` constraint.

**Valid options:**

- EELS1
- HEBI

### Modality

Choose whether to instantiate a worker depending on the `SUBSURFACE` environment variable.

**Valid options:**

- subsurface
- surface

### Change Field

When the `change_field` constraint is specified,
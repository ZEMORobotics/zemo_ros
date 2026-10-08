# Servo Controller

ROS 2 controller for position and closed-loop velocity controlled smart servos.

The controller supports:

- Position controlled servos.
- Closed-loop velocity controlled servos.
- Single-servo writes.
- Multi-servo synchronized writes.
- ROS 2 Action interface for commanding servos.
- Command timeout detection.
- Overcurrent protection.
- Position and velocity goal tolerance checking.

---

## Configuration

```yaml
servo_controller:
  ros__parameters:
    update_rate: Hz

    position_joints: []
    velocity_joints: []

    max_current: mA
    tolerance: rad
    velocity_tolerance: rad/s
```

- `update_rate` — controller update frequency in Hz.
- `position_joints` — list of joints operating in position mode.
- `velocity_joints` — list of joints operating in closed-loop velocity mode.
- `max_current` — maximum allowed motor current in mA.
- `tolerance` — position goal tolerance in radians.
- `velocity_tolerance` — velocity goal tolerance in rad/s.

A joint must not be present in both `position_joints` and `velocity_joints`.

---

## Command Interfaces

### Position mode

```text
position
velocity
acceleration
write_type
sequence
```

- `position` — target position [rad].
- `velocity` — max velocity [rad/s].
- `acceleration` — max acceleration [rad/s²].
- `write_type` — single or synchronized write.
- `sequence` — identifies a new servo command.

### Velocity mode

```text
velocity
acceleration
write_type
sequence
```

- `velocity` — target velocity [rad/s].
- `acceleration` — max acceleration [rad/s²].
- `write_type` — single or synchronized write.
- `sequence` — identifies a new servo command.

---

## State Interfaces

```text
position
velocity
moving
current
```

---

# Action Server

The controller exposes the following ROS 2 action server:

```text
/servo_action_server
```

Action type:

```text
zemo_msgs/action/ServoCommand
```

A goal contains:

```text
names[]
positions[]
velocities[]
accelerations[]
sync_write
timeout_ms
```

All four arrays must have the same size.

Each array index describes one servo:

```text
names[i]
positions[i]
velocities[i]
accelerations[i]
```

For a position-mode servo, `position`, `velocity` and `acceleration` are used.

For a velocity-mode servo, the target is defined by `velocity` and `acceleration`.

`timeout_ms` specifies the maximum amount of time allowed for the command to complete.

The controller rejects a goal if:

The current controller implementation does not allow action cancellation.

---

## Single Servo Command

For a single servo:

```text
sync_write: false
```

Example position command:

```bash
ros2 action send_goal /servo_action_server zemo_msgs/action/ServoCommand \
  "{
    names: ['m1'],
    positions: [1.57],
    velocities: [1.0],
    accelerations: [2.0],
    sync_write: false,
    timeout_ms: 3000
  }"
```

This commands `m1` to approximately 90 degrees (`1.57 rad`).

For a velocity-mode servo:

```bash
ros2 action send_goal /servo_action_server zemo_msgs/action/ServoCommand \
  "{
    names: ['m2'],
    positions: [0.0],
    velocities: [3.0],
    accelerations: [5.0],
    sync_write: false,
    timeout_ms: 3000
  }"
```

The `positions` array must still be supplied because all goal arrays are required to have the same size, but its value is ignored for a velocity-mode servo.

---

## Synchronized Servo Command

Multiple servos can be commanded as a single synchronized command by setting:

```text
sync_write: true
```

Example:

```bash
ros2 action send_goal /servo_action_server zemo_msgs/action/ServoCommand \
  "{
    names: ['m1', 'm2', 'm3'],
    positions: [0.5, 1.0, -0.5],
    velocities: [1.0, 1.0, 1.0],
    accelerations: [2.0, 2.0, 2.0],
    sync_write: true,
    timeout_ms: 3000
  }"
```

This performs a single physical SyncWrite.

---

## Action Completion

### Position mode

A position servo is considered finished when:

```text
moving == 0
```

and:

```text
abs(current_position - target_position) <= tolerance
```

### Velocity mode

A velocity servo is considered finished when:

```text
abs(current_velocity - target_velocity) <= velocity_tolerance
```

For commands containing multiple servos, the action succeeds only when all servos have reached their respective targets.

---

## Action Result

The action returns `SUCCESS` when all requested servos reach their targets.

The goal is aborted if:

- The command exceeds `timeout_ms`.
- Servo current exceeds `max_current`.
- Writing command interfaces fails.

The corresponding result values currently include:

```text
SUCCESS
TIMEOUT
CURRENT
ERROR
```

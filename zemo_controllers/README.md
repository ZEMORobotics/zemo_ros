# Servo Controller Configuration

```yaml
servo_controller:
  ros__parameters:
    update_rate: [hz]

    position_joints: []
    velocity_joints: []

    min_voltage: [V]
    max_voltage: [V]
    max_temperature: [C]
    max_current: [mA]

    default_timeout: [s]
    default_tolerance: [rad]
```

---

- `update_rate` — controller update frequency in Hz.
- `position_joints` — joints operating in position mode.
- `velocity_joints` — joints operating in closed-loop velocity mode.
- `min_voltage` / `max_voltage` — allowed supply voltage range in V.
- `max_temperature` — maximum servo temperature in °C.
- `max_current` — maximum motor current in mA.
- `default_timeout` — default command timeout in seconds.
- `default_tolerance` — default position tolerance in radians.

A joint must not be present in both `position_joints` and `velocity_joints`.

## Command Interfaces

Position mode (0):

```text
position
velocity
acceleration
write_type
sequence
```

Velocity mode (1):

```text
velocity
acceleration
write_type
sequence
```

## State Interfaces

```text
position
velocity
load
voltage
temperature
movement status
current
```
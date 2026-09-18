# ZEMO_HARDWARE

`zemo_hardware` contains the hardware abstraction layer used by `ros2_control`.

---

## Testing

To test the hardware independently from the complete robot bringup, run:

```bash
ros2 launch zemo_description robot_state_publisher_launch.py
```

and in another terminal:

```bash
ros2 launch zemo_hardware hardware_launch.py
```

`controller_manager` receives the robot description from the `robot_description` topic published by `robot_state_publisher`.

---

# CAN Interface

The motion board, IR sensors and cinch are currently connected through a single CAN interface.
According to the `ros2_control` hardware model, all devices sharing this hardware connection are represented by one `SystemInterface`:

```text
CanInterface
├── DiffDrive
├── IRSensor
└── Cinch
```

- There should be only one user space proccess accessing CAN communication.

- `CanInterface` owns the Linux SocketCAN socket and is responsible for it.

- Individual hardware classes are responsible for interpreting their own protocol.

- This keeps the CAN transport separated from device-specific logic and makes adding new CAN devices simple (OPEN/CLOSED principle).

## Diff Drive

The differential drive is controlled by the standard ros2 control:

```text
diff_drive_controller/DiffDriveController
```

The hardware interface exposes:

```text
wheel_left_joint
    command: velocity
    state:   velocity
    state:   position

wheel_right_joint
    command: velocity
    state:   velocity
    state:   position
```

IR sensors and cinch input are simple state devices and currently do not use a dedicated `ros2_control` controller.

## CAN Protocol

- The system uses **Classical CAN** with **11-bit standard CAN identifiers**.

- All multi-byte numeric values are encoded in **big-endian byte order**.

- Messages containing values for both wheels use the complete 8-byte CAN payload:

Layout:

```text
Byte:
  0    1    2    3    4    5    6    7
+----+----+----+----+----+----+----+----+
|      LEFT VALUE   |     RIGHT VALUE   |
+----+----+----+----+----+----+----+----+
      32-bit               32-bit
      big endian           big endian
```

CAN IDs are defined in:

```text
can_protocol.hpp
```

| CAN ID | Name | Direction | Payload |
|---:|---|---|---|
| `100` | `WHEEL_COMMAND_VEL` | Raspberry → Motion board | Left `float32` + Right `float32` |
| `101` | `WHEEL_FEEDBACK_VEL` | Motion board → Raspberry | Left `float32` + Right `float32` |
| `102` | `WHEEL_POSITION` | Motion board → Raspberry | Left encoder `int32` + Right encoder `int32` |
| `103` | `PID_KP` | Raspberry → Motion board | `float32` |
| `104` | `PID_KI` | Raspberry → Motion board | `float32` |
| `105` | `PID_KD` | Raspberry → Motion board | `float32` |
| `106` | `PID_KP_ROT` | Raspberry → Motion board | `float32` |
| `107` | `PID_KI_ROT` | Raspberry → Motion board | `float32` |
| `108` | `PID_KD_ROT` | Raspberry → Motion board | `float32` |
| `109` | `MOTORS_ON` | Raspberry → Motion board | `int32` command |
| `110` | `MOTORS_OFF` | Raspberry → Motion board | `int32` command |
| `111` | `FEEDBACK_ON` | Raspberry → Motion board | `int32` command |
| `112` | `FEEDBACK_OFF` | Raspberry → Motion board | `int32` command |
| `113` | `REFERENCE_ON` | Raspberry → Motion board | `int32` command |
| `114` | `REFERENCE_OFF` | Raspberry → Motion board | `int32` command |
| `115` | `RESET_ENCODERS` | Raspberry → Motion board | `int32` command |
| `116` | `RESET_PID` | Raspberry → Motion board | `int32` command |
| `150` | `FRONT_IR` | Hardware → Raspberry | IR state |
| `151` | `BACK_IR` | Hardware → Raspberry | IR state |
| `152` | `CINCH` | Hardware → Raspberry | Cinch state |

## Example: wheel velocity command

Move slowly forward:

```

ros2 topic pub -r 10 /diff_drive_controller/cmd_vel  geometry_msgs/msg/TwistStamped \
"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

```

If the robot is not moving forward, you need to change pin directions.
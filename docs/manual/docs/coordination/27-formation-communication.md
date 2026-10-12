# 27. Formation control with a communication model

!!! success "Ported and tested"
    Package `xtdrone_formation_demo`: `comm_model.py` contains the SINR channel model (`communication_verify_1.py`),
    the per-UAV consensus controller (`sim_addleader_1.py`) and the UAV-state aggregator. Messages are in the new
    package `xtdrone_msgs`. Scenario `formation_comm_20`. Test: `scripts/tests/run_chapter_tests.sh ch27`: 20 iris
    take off; in FORM_1 the leader moves 12 m, UAVs 1–12 follow within 0.65 m of its displacement, and UAV 13,
    isolated by the channel model, holds its position (0.02 m).

Twenty iris keep a formation while a wireless channel model decides which vehicles can hear each other. Every UAV
transmits its state. A receiver decodes a transmitter only if the channel capacity B·log2(1 + SINR) exceeds the
required rate (128 bytes × N UAVs × 15 Hz). The SINR uses the channel gain 1 / (1 + d²) and counts all other
transmitters as interference. Each UAV runs a second-order consensus controller over the neighbours it hears;
UAV 0 is the leader.

With all 20 transmitting, the main group (UAVs 0–12) hears itself well. UAVs 13–16 hear only each other, and UAV 13
hears no lower-numbered UAV, so it acts as their local leader and holds its position. UAVs 17–19 hear a few vehicles
of the main group.

## Run

```bash
# 1. twenty iris (the start positions are formation table UAV_pos1)
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=formation_comm_20
# 2. channel model, aggregator and the 20 controllers (in one process)
ros2 launch xtdrone_formation_demo formation_communication.launch.py
# 3. keyboard: g (control the leader), v (AUTO.TAKEOFF), 1 (FORM_1), 2 (FORM_2), s (HOVER), n (AUTO.LAND)
ros2 run xtdrone_py multirotor_keyboard_control iris 20 vel
```

`AUTO.TAKEOFF` arms all vehicles and climbs to 0.6 m, as in ROS 1. In `FORM_1` the leader moves along +y
(`FORM_2`: −y) and the others keep their relative positions from the table. Instead of the keyboard, XTDGroundControl
(chapter 42) or `/xtdrone_gcs/cmd` can send the same commands. Do not start the communication nodes
(`communication:=none`, the default): the controllers command MAVROS directly.

| `formation_communication.launch.py` argument | Default | Meaning |
|---|---|---|
| `uav_count` | `20` | number of UAVs |
| `leaders` | `all` | transmitting UAVs, e.g. `0,13,17` (the `leader_id` variants of `communication_verify_1.py`) |
| `pos_file` | `UAV_pos1` | desired formation table `UAV_pos1`, `UAV_pos2`, `UAV_pos3` or a path |
| `single_process` | `true` | all nodes in one process; `false` starts one process per node, like `mpiexec` |

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/iris_<i>/formation/uav_data` | `xtdrone_msgs/UAVDataMsg` | controller → aggregator |
| `/xtdrone_gcs/all_uav_data` | `xtdrone_msgs/AllUAVData` | aggregator → model, controllers |
| `/communication_verify` | `xtdrone_msgs/CommVerify` (`trans_matrix[i].trans_matrix[k]`: UAV i hears leader k) | model → controllers |
| `/xtdrone/leader/cmd`, `/xtdrone/iris_1/cmd`, `/xtdrone_gcs/cmd` | String | keyboard / GUI → controllers |
| `/iris_<i>/mavros/setpoint_raw/local` | PositionTarget | controller → MAVROS |

!!! note "Changes from the ROS 1 version"
    - No MPI: the controllers are rclpy nodes, by default in one process. The UAV-state aggregation, which the
      modified keyboard or `xtdrone_qt` did in ROS 1, is a separate node, so the stock keyboard works.
    - Namespaces `iris_<i>` instead of `UAV<i>`. The message field `transMatrix` is `trans_matrix` (ROS 2 field
      names are snake_case).
    - UAVs 13–19 start at the positions their controller assumes (`uav_bias`). The ROS 1 launch file put them at
      y = ±9, which did not match.
    - All setpoints use `FRAME_LOCAL_NED` (1). ROS 1 sent position targets with frame 7 (`LOCAL_OFFSET_NED`).

## Twenty vehicles on one machine

Twenty PX4 instances with MAVROS are heavy. On the 6-core test machine without a GPU, Gazebo reaches about 0.3 of real
time. More than six vehicles automatically use the `swarm` MAVLink profile and MAVROS plugin set (chapter 43).
Also run `scripts/tune_network_for_swarms.sh` once. It reserves the PX4 MAVLink ports and enlarges the socket
buffers that DDS discovery needs with many processes; without it, single vehicles can stay unreachable.

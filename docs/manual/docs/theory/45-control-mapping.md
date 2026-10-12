# 45. Electro-mechanical model: from control commands to actuators

!!! success "Rewritten for PX4 v1.16 and Gazebo Harmonic"
    Background chapter. The mixers and the `gazebo_mavlink_interface` plugin of the original no longer exist.

## Background

- **Control allocation**: since PX4 1.13, a control allocation module replaces the mixer files. It turns the
  torque and thrust demand of the controllers into normalised motor and servo commands. Each airframe describes
  its geometry with parameters: `CA_AIRFRAME`, `CA_ROTOR_COUNT`, and `CA_ROTORi_PX/PY/KM` for position and
  moment coefficient. See the [PX4 actuator configuration](https://docs.px4.io/v1.16/en/config/actuators.html).
- **uORB**: PX4's internal publish/subscribe bus, similar to ROS topics.

## Flight hardware

Commands from RC, MAVLink or offboard setpoints → `commander` / `mavlink` → controllers → `vehicle_torque_setpoint`
and `vehicle_thrust_setpoint` → **control allocation** → `actuator_motors` / `actuator_servos` → output driver
(PWM, DShot, UAVCAN).

## SITL with Gazebo Harmonic

In SITL, the output driver is PX4's **`gz_bridge`** module. Gazebo Classic needed the `gazebo_mavlink_interface`
plugin and MAVLink `HIL_ACTUATOR_CONTROLS` messages. `gz_bridge` instead publishes the actuator values directly on
gz-transport:

| Output | gz topic | PX4 parameters | Gazebo side |
|---|---|---|---|
| motors (ESC) | `/<model>/command/motor_speed` (`gz.msgs.Actuators`, rad/s) | `SIM_GZ_EC_FUNCi`, `SIM_GZ_EC_MINi`, `SIM_GZ_EC_MAXi` | `MulticopterMotorModel` system per rotor, `<motorNumber>` = index |
| servos | `/model/<model>/<joint name>` (`gz.msgs.Double`, rad) | `SIM_GZ_SV_FUNCi`, `SIM_GZ_SV_MINAi`, `SIM_GZ_SV_MAXAi` (angle limits) | `JointPositionController` per control surface |
| wheels | `/model/<model>/command/motor_speed` | `SIM_GZ_WH_*` | wheel joint controllers |

The mapping is linear. For a motor with function `SIM_GZ_EC_FUNCi = 101 + k` (motor k+1), a normalised command
u ∈ [0, 1] becomes

\[
\omega_\text{ref} = \text{SIM\_GZ\_EC\_MIN}_i + u\,(\text{SIM\_GZ\_EC\_MAX}_i - \text{SIM\_GZ\_EC\_MIN}_i)\quad[\text{rad/s}]
\]

so `SIM_GZ_EC_MIN` sets the armed idle speed and `SIM_GZ_EC_MAX` the full-throttle speed. The old
`<input_scaling>`, `<zero_position_armed>` and `<input_index>` settings map to these parameters.

## Example: the XTDrone typhoon_h480

PX4 v1.16 has no Gazebo hexacopter, so XTDrone generates the airframe `4901_gz_typhoon_h480` from the model
(`generate_vehicle_models.py`):

```sh
param set-default CA_ROTOR_COUNT 6
param set-default CA_ROTOR0_PX -0.2094     # rotor_0 link position, FRD frame
param set-default CA_ROTOR0_PY -0.1228
param set-default CA_ROTOR0_KM -0.05       # cw rotor
...
param set-default SIM_GZ_EC_FUNC1 101      # actuator index 0 = Motor 1 = <motorNumber>0
param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MAX1 1500      # = <maxRotVelocity> of the Classic model
```

The generator numbers the rotors in link order. It writes the same index into `<motorNumber>` of each
`MulticopterMotorModel` and into the `CA_ROTORi` entries, so allocation and simulation always agree.

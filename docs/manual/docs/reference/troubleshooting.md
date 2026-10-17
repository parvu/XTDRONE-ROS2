# Troubleshooting

## `connected: false` in `/<n>/mavros/state`

- Wait 10–20 s after the vehicle appears in Gazebo: PX4 waits for the world, then starts its MAVLink links.
- Check that PX4 runs: the launch terminal shows `INFO [init] PX4_GZ_MODEL_NAME set, PX4 will attach to existing model`
  and later `Ready for takeoff!`.
- Check the ports (chapter 48): `ss -ulpn | grep 2454`. Another program, or a leftover PX4 from an earlier run, may hold
  them: `pkill -x px4`.
- With many vehicles (about 20 on 6 cores), one MAVROS node can stall at startup: PX4 reports `Ready for takeoff!`
  and MAVROS logs `link[1000] detected remote address`, but after one `VER: broadcast request timeout` it logs nothing
  more and never connects. This was seen once in the 20-vehicle test (ch27); the re-run passed. Restart the launch.

## `gz_bridge failed to start and attach to existing model`

The model with that name is not in the world. Check `gz model --list`. A spawn fails when the model file is not
found. Check `GZ_SIM_RESOURCE_PATH`, and run `scripts/build_xtdrone_ros2.sh` again if `share/xtdrone_gz/classic_models`
is missing.

## PX4 refuses to arm

Read the reason in the launch terminal (`Preflight Fail: ...`) or with `ros2 topic echo /<n>/mavros/statustext/recv`.

- *No valid position / estimator not ready*: with `ekf:=vision`, start `get_local_pose` or your SLAM first.
- *Compass / mag*: the world needs the `Magnetometer` system. All XTDrone worlds have it. Add it to your own worlds.
- *Offboard not available*: the communication node must stream setpoints **before** you switch to OFFBOARD; start it first.

## Arming denied: "No connection to the ground control station"

PX4 v1.16 airframes set `NAV_DLL_ACT=2`. `xtdrone.launch.py` sets it to 0 for every vehicle. If you start PX4
another way, or pass `px4_params:="NAV_DLL_ACT=2"`, connect QGroundControl first.

## Arming denied right after start

"Resolve system health failures first" during the first seconds after the home position is set is normal:
the estimator is still settling. Arm again a few seconds later. The XTDrone communication node ignores a
command identical to the previous one, so press `b` (OFFBOARD) and then `t` (ARM) again.

## Black or untextured models

The world references a model that was not converted, or a texture under `file://media/...`. Look for `[Err]` lines in
the launch output, add the model directory to `--model-path` of `classic_to_gz.py`, and rebuild.

## Simulation slower than real time

Camera and lidar sensors are expensive, more so without a GPU. On WSL2, check that Gazebo uses the Windows GPU:
`source scripts/xtdrone_env.sh` sets `GALLIUM_DRIVER=d3d12`; if you start Gazebo without that script, Mesa renders on
the CPU (`glxinfo -B | grep renderer` says `llvmpipe`) and a lidar or SLAM simulation runs at a few percent of real time. Images are only converted while a node
subscribes, so close image viewers and `ros2 topic echo` on image topics you do not need. Use vehicles without the sensors you do not need,
`gui:=false`, or fewer vehicles. Check the real time factor with `gz topic -e -t /stats`.

## The vehicle is not visible in the Gazebo GUI

- Check that it exists on the server: `gz model --list` must list `iris_0`. Then find it in the GUI's
  *Entity tree*, right-click it and choose *Move to*. The original Classic camera poses are far away
  (e.g. `indoor1`: 20 m), so a 0.5 m drone is easy to miss.
- If the entity tree does not list it, the GUI missed its creation: close the GUI and run `gz sim -g` again.
  `xtdrone.launch.py` starts the GUI only after the vehicles are spawned to avoid this.

## Gazebo GUI crashes with `symbol lookup error: /snap/...`

The terminal runs inside a snap (e.g. the VS Code snap) and passes the snap's library paths on.
`source scripts/xtdrone_env.sh` removes them; or use a normal terminal.

## Leftover processes

```bash
pkill -f "gz sim"; pkill -x px4; pkill -f 'etc/init.d-posix/rc[S]'; pkill -f mavros_node; pkill -f parameter_bridge
```

## `ros2` fails after sourcing another ROS distribution

ROS environments must not be mixed. Open a new terminal and use `source scripts/xtdrone_env.sh`, which removes
any other ROS distribution and a Python venv from the shell.

## PX4 build fails with Python errors

The build must use the system Python. `scripts/build_xtdrone_ros2.sh` leaves an active virtualenv automatically.
If you build PX4 by hand, run `deactivate` first.

## PX4: "gz_bridge Task already running … failed to attach to existing model"

A PX4 from an earlier launch was killed while its startup script (`rcS`) was still waiting for Gazebo. The orphaned
shell keeps running and later sends its commands to the new PX4 with the same instance number. Kill it before
starting again:

```bash
pkill -f 'etc/init.d-posix/rc[S]'
```

## Large fleets: single vehicles never connect, "bind failed: Address already in use", "HEARTBEAT timed out"

Run `scripts/tune_network_for_swarms.sh` once (reserved PX4 MAVLink ports, larger socket buffers for DDS
discovery). See chapter 43 for what else keeps 10+ vehicles running on one machine.

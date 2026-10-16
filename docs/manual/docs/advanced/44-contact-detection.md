# 44. UAV contact detection

!!! success "Ported and tested"
    `ros2 run xtdrone_py contact_detect`. Test: `scripts/tests/run_chapter_tests.sh ch44`.

Some tasks need to know whether a UAV touches something. Like `bumper_sensor_iris` in Classic, every XTDrone
vehicle model for Gazebo Harmonic carries a **bumper**: a light box around the frame with a gz `contact`
sensor. The launch bridges it to

```text
/<type>_<id>/benchmarker/collision     ros_gz_interfaces/msg/Contacts
```

A Gazebo contact sensor only reports contacts of its own link, so the bumper is a separate link
(`bumper_link`), fixed to `base_link`:

| Vehicle | Bumper box | Position relative to `base_link` |
|---|---|---|
| iris / solo (x500 frame) | 0.65 × 0.65 × 0.31 m | z = −0.072 m: from the bottom of the legs to above the body |
| typhoon_h480 | 0.7 × 0.7 × 0.35 m | z = −0.085 m |

For your own vehicles, change `kit_bumper(size=..., z=...)` in `ros2/xtdrone_gz/scripts/generate_vehicle_models.py`.

## Run

Six iris on the ground:

```bash
ros2 launch xtdrone_bringup xtdrone.launch.py world:=empty vehicles:="iris*6"
ros2 run xtdrone_py contact_detect iris 6
```

While the vehicles stand on the ground, each one reports a contact with the ground plane:

```text
iris_0 contact ground_plane::link::collision
One contact position is [0.315, 2.666, -0.001]
...
```

After takeoff the messages stop, until a vehicle touches an obstacle.

## Visualise

```bash
ros2 topic echo /iris_0/benchmarker/collision       # a Contacts message appears on each collision (silent otherwise)
```

`contact_detect` prints a line when a vehicle touches something. To test it, fly a vehicle into the ground or a
wall; Gazebo shows the contact points with View → Contacts.

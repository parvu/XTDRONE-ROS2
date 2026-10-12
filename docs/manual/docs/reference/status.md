# Chapter status

Legend: **Tested** = ported, and the test case in `scripts/tests/run_chapter_tests.sh` passed on Ubuntu 24.04 /
ROS 2 Jazzy / Gazebo Harmonic 8.15 / PX4 v1.16.2 / MAVROS 2.16 (6-core CPU, no GPU), last full run 2026-10-12. **Ported** = converted, not (yet) covered by a passing test.
**Partial** = part of the chapter works. **Planned** = waiting for a later migration phase.

| # | Chapter | Status | Test case | Phase |
|---|---|---|---|---|
| 1 | Video introduction | No porting needed | — | — |
| 2–3 | Basic configuration | **Tested** | `ch03` | 1 |
| 4 | Docker | Ported (image not built on the test host) | — | 1 |
| 5 | Single-direction position hold | **Tested** | `ch05` | 1 |
| 6 | PX4 EKF configuration | **Tested** | `ch06` | 1 |
| 7 | Attitude control | **Tested** (both modes) | `ch07` | 1 |
| 8 | Different UAV types | **Tested** (iris, plane, standard_vtol, rover; typhoon_h480, tailsitter, tiltrotor take off in the PX4-level check) | `ch08plane`, `ch08vtol`, `ch08rover` | 1 |
| 9 | Multi-vehicle | **Tested** (6 iris) | `ch09` | 1 |
| 10 | Add a vehicle at runtime | **Tested** | `ch10` | 1 |
| 11 | UGV | Planned | — | 4 |
| 12 | Sensor configuration | **Tested** | `ch12` | 2 |
| 13 | Visual SLAM | Planned | — | 5 |
| 14 | VIO | Planned | — | 5 |
| 15 | Ground-truth pose | **Tested** | `ch15` | 1 |
| 16 | Dense reconstruction | Planned | — | 4 |
| 17 | 2D laser SLAM (Hector → slam_toolbox) | Planned | — | 4 |
| 18 | 2D laser SLAM (Cartographer) | Planned | — | 4 |
| 19 | 3D laser SLAM | Planned | — | 5 |
| 20 | 2D motion planning | Planned | — | 4 |
| 21 | 3D motion planning | Planned | — | 5 |
| 22 | Swarm motion planning | Planned | — | 5 |
| 23 | Detection and tracking | Planned (gimbal camera available) | — | 5 |
| 24 | Formation | Planned | — | 3 |
| 25 | Formation (ROS 2) | **Tested** (6 iris: waiting, T, triangle, moving leader) | `ch25` | 3 |
| 26 | Formation HIL | Planned | — | 3 |
| 27 | Formation with communication model | Planned | — | 3 |
| 28 | Precision landing | Planned (scenario, camera and tags available) | — | 4 |
| 29 | QGC waypoints for many UAVs | Ported | — | 1 |
| 30 | Self-driving car | Planned | — | 4 |
| 31 | UGV planning | Planned | — | 5 |
| 32 | UAV + USV | Planned | — | 4 |
| 33 | Aerial robotic arm | Planned | — | 5 |
| 34 | Fixed-wing affine formation | Planned | — | 5 |
| 35 | Fixed-wing formation tracking a UGV | Planned | — | 5 |
| 36 | Flat models | **Tested** | `ch36` | 2 |
| 37 | Moving environment models | **Tested** | `ch37` | 2 |
| 38 | Actors | Partial: scripted actors and start positions; ROS control planned | `worlds` | 2 / 4 |
| 39 | Building scenes | Ported (converter workflow, tested on all XTDrone worlds) | `worlds` | 2 |
| 40 | Large scenes | Ported (heightmap worlds `outdoor1`, `robocup` load) | `worlds` | 2 |
| 41 | Wind | **Tested** | `ch41` | 2 |
| 42 | XTDGroundControl | Planned | — | 3 |
| 43 | Acceleration, large swarms | **Tested** | `ch43` | 2 |
| 44 | Contact detection | **Tested** | `ch44` | 2 |
| 45–48 | Theory | Rewritten | — | — |
| — | All XTDrone worlds load (20 converted + `empty`, `empty_zero_g`) | **Tested** | `worlds` | 2 |

Run all tests with `scripts/tests/run_chapter_tests.sh` (chapter [Testing](testing.md)).

# 26. UAV formation hardware-in-the-loop simulation

!!! info "Ported, not tested on hardware"
    The nodes are those of chapters 24 and 25. They do not change when they run on other computers. What changes
    is the networking: ROS 2 has no master, so the ROS 1 hosts/`ROS_MASTER_URI` setup is replaced by DDS discovery.
    The test machine has no development boards, so the multi-machine part is not tested. The same launch on one
    machine is covered by the chapter 24/25 tests.

To be closer to real flight, the formation nodes run on development boards instead of the simulation computer. The
original experiment used six vehicles: a laptop runs XTDrone, an NVIDIA NX runs the leader, and two Raspberry Pis
plus three ODROIDs run the five followers.

## Network setup (replaces hosts files and ROS_MASTER_URI)

All computers must be in one local network and run ROS 2 Jazzy, or another ROS 2 distribution that uses the same
messages; `xtdrone_formation_demo` needs only `std_msgs`, `geometry_msgs` and NumPy. Set the same domain on every
computer, for example in `~/.bashrc`:

```bash
export ROS_DOMAIN_ID=26                       # any 0-101, the same on all computers
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET   # default; LOCALHOST would keep the nodes on one machine
```

There is no master to start. Nodes find each other by multicast. Check it with `ros2 multicast receive` on one
computer and `ros2 multicast send` on another. If multicast is blocked (some Wi-Fi access points), list the peers
instead:

```bash
export ROS_STATIC_PEERS="192.168.1.10;192.168.1.29;192.168.1.30"   # IPs of the other computers
```

Hostnames and `/etc/hosts` entries are not needed. For remote control of the boards, SSH plus tmux works as
before (`sudo apt install openssh-server tmux`).

## Formation experiment

On the simulation computer:

```bash
# six iris on the formation_6 grid, ground truth for EKF2 (one shared frame, chapter 15)
ros2 launch xtdrone_bringup xtdrone.launch.py scenario:=formation_6 ekf:=vision communication:=basic
ros2 run xtdrone_py get_local_pose iris 6
ros2 run xtdrone_py multirotor_keyboard_control iris 6 vel   # take off all, climb to about 8 m, hover
```

`communication:=basic` starts the six communication nodes on the simulation computer. To run them on the boards as in
the original setup, leave it out and start `ros2 run xtdrone_py multirotor_communication iris <i>` on board `<i>`.

On the leader board (NX), then on each follower board, with the package built there
(`colcon build --packages-select xtdrone_formation_demo`):

```bash
ros2 run xtdrone_formation_demo leader iris 6                 # NX
ros2 run xtdrone_formation_demo follower iris 1 6             # board 1 (followers 1 ... 5)
```

In ROS 1 the followers took two arguments (`follower.py iris 1`). The ROS 2 port also takes the fleet size. Then
press `g` in the keyboard window and switch formations with the digit keys, as in chapter 25.

!!! tip "Bandwidth"
    The followers subscribe to the MAVROS poses of their vehicle and the leader at about 30 Hz each, which is
    little traffic. Do not bridge camera images when the boards are on Wi-Fi: the bridges are lazy (chapter 12)
    and send images only to subscribers, so simply do not subscribe to them from the boards.

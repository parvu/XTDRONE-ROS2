#!/usr/bin/env bash
# Kernel network settings for large XTDrone fleets (10+ vehicles on one machine).
#   ./scripts/tune_network_for_swarms.sh      (asks for the sudo password; persists across reboots)
# - PX4 binds its MAVROS link on UDP 34580+N, inside the kernel's ephemeral port range; reserve
#   those ports so that no random outgoing socket takes one first.
# - Larger socket receive buffers: every DDS participant receives every discovery multicast; with
#   the 208 KB default many are dropped and topics between processes take minutes to connect.
set -euo pipefail
sudo tee /etc/sysctl.d/60-xtdrone.conf >/dev/null <<'SYSCTL'
net.ipv4.ip_local_reserved_ports = 34580-34779
net.core.rmem_max = 33554432
net.core.rmem_default = 8388608
net.core.wmem_max = 33554432
net.core.wmem_default = 8388608
SYSCTL
sudo rm -f /etc/sysctl.d/60-xtdrone-ports.conf
sudo sysctl -p /etc/sysctl.d/60-xtdrone.conf

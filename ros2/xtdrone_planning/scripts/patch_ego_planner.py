#!/usr/bin/env python3
"""Put the EGO-Planner ROS 2 port (plan_manage) on the node clock, so it follows simulation time.

  patch_ego_planner.py <copy of ego-planner-swarm/src/planner/plan_manage>

The port times its trajectories with rclcpp::Clock() (wall time) where ROS 1 used ros::Time::now()
(simulation time with use_sim_time). With Gazebo slower than real time the trajectory server then runs
ahead of the vehicle, and the planner replans from where the trajectory says the vehicle should be, past
the obstacles. This adds a process-wide clock (ego_planner/ego_clock.h) that main() sets to the node
clock, and uses it in the FSM, the planner manager and the trajectory server. Applied by
scripts/build_xtdrone_ros2.sh to a copy in the workspace; the submodule is not modified. Idempotent.
"""
import pathlib
import sys

HEADER = """// Added by xtdrone_planning/scripts/patch_ego_planner.py: one clock for the whole process, set to the
// node clock in main(), so that EGO-Planner follows simulation time (use_sim_time) like in ROS 1.
#pragma once
#include <rclcpp/rclcpp.hpp>

namespace ego_planner
{
inline rclcpp::Clock::SharedPtr & egoClock()
{
  static rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  return clock;
}
inline rclcpp::Time egoNow() { return egoClock()->now(); }
}  // namespace ego_planner
"""


def edit(path, pairs, include_after=None):
    s = path.read_text()
    if "ego_planner/ego_clock.h" in s:
        return
    for old, new in pairs:
        if old not in s:
            raise SystemExit(f"{path}: '{old}' not found; the EGO-Planner port changed, update this patch")
        s = s.replace(old, new)
    if include_after:
        s = s.replace(include_after, include_after + "\n#include <ego_planner/ego_clock.h>", 1)
    path.write_text(s)


def main():
    root = pathlib.Path(sys.argv[1])
    (root / "include" / "ego_planner" / "ego_clock.h").write_text(HEADER)
    now = ("rclcpp::Clock().now()", "ego_planner::egoNow()")
    # Swarm sequential start: each planner sends its trajectories to the next one once, on
    # /drone_<id>_planning/swarm_trajs. With slow DDS discovery on a loaded machine the next planner's
    # subscription may match only after that message: transient-local QoS hands it the last one.
    latched = "rclcpp::QoS(10).reliable().transient_local()"
    edit(root / "src" / "ego_replan_fsm.cpp",
         [now, ("rclcpp::Clock clock(RCL_SYSTEM_TIME);", "rclcpp::Clock clock(RCL_ROS_TIME);"),
          ("create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);",
           f"create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, {latched});"),
          ("create_subscription<traj_utils::msg::MultiBsplines>(\n          sub_topic_name,\n          10,",
           f"create_subscription<traj_utils::msg::MultiBsplines>(\n          sub_topic_name,\n          {latched},"),
          # fsm/sequential_start (default true, the upstream behaviour): false lets every planner start
          # without the trajectories of the one before it; they still avoid each other through
          # /broadcast_bspline. XTDrone's swarm launch sets it false: on a loaded machine the one-off hand-over
          # was lost in about half of the runs and left vehicles waiting for ever.
          ("      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))",
           "      static const bool sequential = [this] {\n"
           "        if (!node_->has_parameter(\"fsm/sequential_start\"))\n"
           "          node_->declare_parameter(\"fsm/sequential_start\", true);\n"
           "        return node_->get_parameter(\"fsm/sequential_start\").as_bool();\n"
           "      }();\n"
           "      if (!sequential || planner_manager_->pp_.drone_id <= 0 ||\n"
           "          (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))")],
         include_after="#include <ego_planner/ego_replan_fsm.h>")
    edit(root / "src" / "planner_manager.cpp", [now], include_after="#include <ego_planner/planner_manager.h>")
    edit(root / "src" / "ego_planner_node.cpp",
         [("  rebo_replan.init(node);", "  ego_planner::egoClock() = node->get_clock();\n  rebo_replan.init(node);")],
         include_after="#include <ego_planner/ego_replan_fsm.h>")
    edit(root / "src" / "traj_server.cpp",
         [("rclcpp::Clock clock(RCL_ROS_TIME);", "// node clock: ego_planner::egoNow()"),
          ("clock.now()", "ego_planner::egoNow()"),
          ('  auto node = rclcpp::Node::make_shared("traj_server");',
           '  auto node = rclcpp::Node::make_shared("traj_server");\n  ego_planner::egoClock() = node->get_clock();')],
         include_after="#include <rclcpp/rclcpp.hpp>")
    print(f"patched {root}")


if __name__ == "__main__":
    main()

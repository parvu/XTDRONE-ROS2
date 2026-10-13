// vir_sim_leader (chapter 35), ROS 2 port: publishes the leader's state (FwStates of plane leader_id) as
// LeaderStates for the followers' formation controllers, at 10 Hz as in ROS 1.
//   ros2 run xtdrone_fixed_wing_formation vir_sim_leader
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "xtdrone_fixed_wing_formation/msg/fw_states.hpp"
#include "xtdrone_fixed_wing_formation/msg/leader_states.hpp"

namespace msg = xtdrone_fixed_wing_formation::msg;

class VirSimLeader : public rclcpp::Node {
public:
  VirSimLeader() : Node("vir_sim_leader") {
    const auto prefix = declare_parameter<std::string>("vehicle_prefix", "standard_vtol_");
    const std::string ns = "/" + prefix + std::to_string(declare_parameter<int>("leader_id", 0)) + "/";
    pub_ = create_publisher<msg::LeaderStates>(ns + "fixed_wing_formation_control/leader_states", 10);
    sub_ = create_subscription<msg::FwStates>(ns + "fixed_wing_formation_control/fw_states", 10,
        [this](const msg::FwStates::SharedPtr m) { fw_ = *m; have_ = true; });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!have_) return;
      msg::LeaderStates l;
      l.altitude = fw_.altitude;
      l.latitude = fw_.latitude;
      l.longitude = fw_.longitude;
      l.ned_vel_x = fw_.ned_vel_x;
      l.ned_vel_y = fw_.ned_vel_y;
      l.ned_vel_z = fw_.ned_vel_z;
      l.global_vel_x = fw_.global_vel_x;
      l.global_vel_y = fw_.global_vel_y;
      l.global_vel_z = fw_.global_vel_z;
      l.roll_angle = fw_.roll_angle;
      l.pitch_angle = fw_.pitch_angle;
      l.yaw_angle = fw_.yaw_angle;
      l.relative_alt = fw_.relative_alt;
      l.pitch_rate = fw_.pitch_rate;
      l.roll_rate = fw_.roll_rate;
      l.yaw_rate = fw_.yaw_rate;
      // the ROS 1 node left the airspeed at 0, and task_main passes it to the controller as the leader's speed
      l.airspeed = fw_.air_speed;
      pub_->publish(l);
    });
  }

private:
  msg::FwStates fw_;
  bool have_ = false;
  rclcpp::Publisher<msg::LeaderStates>::SharedPtr pub_;
  rclcpp::Subscription<msg::FwStates>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirSimLeader>());
  rclcpp::shutdown();
  return 0;
}

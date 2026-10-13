// switch_fw_mode (chapter 35), ROS 2 port: puts one fixed-wing vehicle into formation mode. ROS 1 read
// "<mode> <formation>" from the keyboard (mode 1 OFFBOARD + formation, 2 AUTO.MISSION; formation 1 north-south
// line, 2 triangle, 3 east-west line); here they are parameters:
//   ros2 run xtdrone_fixed_wing_formation switch_fw_mode --ros-args -p plane_id:=1 -p mode:=1 -p formation_type:=2
// It publishes FwCmdMode (need_formation, formation_type) and switches PX4's mode until it is set.
#include <memory>
#include <string>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "xtdrone_fixed_wing_formation/msg/fw_cmd_mode.hpp"

namespace msg = xtdrone_fixed_wing_formation::msg;

class SwitchFwMode : public rclcpp::Node {
public:
  SwitchFwMode() : Node("switch_fw_mode") {
    const int plane_id = declare_parameter<int>("plane_id", 1);
    const auto prefix = declare_parameter<std::string>("vehicle_prefix", "standard_vtol_");
    const int mode = declare_parameter<int>("mode", 1);
    const std::string ns = "/" + prefix + std::to_string(plane_id) + "/";
    cmd_mode_.plane_id = plane_id;
    if (mode == 1) {
      px4_mode_ = "OFFBOARD";
      cmd_mode_.need_formation = true;
      cmd_mode_.formation_type = declare_parameter<int>("formation_type", 1);
    } else {
      px4_mode_ = "AUTO.MISSION";
    }
    state_sub_ = create_subscription<mavros_msgs::msg::State>(ns + "mavros/state", rclcpp::SensorDataQoS(),
        [this](const mavros_msgs::msg::State::SharedPtr m) { state_ = *m; });
    pub_ = create_publisher<msg::FwCmdMode>(ns + "fixed_wing_formation_control/fw_cmd_mode", 10);
    set_mode_ = create_client<mavros_msgs::srv::SetMode>(ns + "mavros/set_mode");
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      pub_->publish(cmd_mode_);  // kept up: task_main acts on the latest FwCmdMode
      if (state_.mode != px4_mode_ && set_mode_->service_is_ready() && ++tick_ % 5 == 0) {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = px4_mode_;
        set_mode_->async_send_request(req);
        RCLCPP_INFO(get_logger(), "mode %s -> %s", state_.mode.c_str(), px4_mode_.c_str());
      }
    });
  }

private:
  msg::FwCmdMode cmd_mode_;
  std::string px4_mode_;
  int tick_ = 0;
  mavros_msgs::msg::State state_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<msg::FwCmdMode>::SharedPtr pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwitchFwMode>());
  rclcpp::shutdown();
  return 0;
}

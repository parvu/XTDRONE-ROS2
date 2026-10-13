// task_main (chapter 35), ROS 2 port of XTDrone's coordination/fixed_wing_formation_control task_main: the
// mission state machine of one fixed-wing vehicle. In formation mode a follower runs the formation controller
// (ABS_FORMATION_CONTROLLER with TECS) on its own state (FwStates) and the leader's (LeaderStates) and sends
// attitude and throttle (FwCmd) to pack_fw_states; the leader only asks for OFFBOARD.
//   ros2 run xtdrone_fixed_wing_formation task_main --ros-args -p plane_id:=1
// The ROS 1 version had one executable per plane (leader_main, follower1_main, ...).
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "xtdrone_fixed_wing_formation/msg/formation_control_states.hpp"
#include "xtdrone_fixed_wing_formation/msg/fw_cmd.hpp"
#include "xtdrone_fixed_wing_formation/msg/fw_cmd_mode.hpp"
#include "xtdrone_fixed_wing_formation/msg/fw_current_mode.hpp"
#include "xtdrone_fixed_wing_formation/msg/fw_monitor.hpp"
#include "xtdrone_fixed_wing_formation/msg/fw_states.hpp"
#include "xtdrone_fixed_wing_formation/msg/leader_states.hpp"
#include "../fixed_wing_lib/syslib.hpp"
#include "../formation_controller/abs_formation_controller.hpp"
#include "../formation_controller/formation_controller.hpp"

namespace msg = xtdrone_fixed_wing_formation::msg;

class TaskMain : public rclcpp::Node {
public:
  TaskMain() : Node("task_main") {
    plane_id_ = declare_parameter<int>("plane_id", 1);
    const auto prefix = declare_parameter<std::string>("vehicle_prefix", "standard_vtol_");
    const std::string ns = "/" + prefix + std::to_string(plane_id_) + "/";
    const std::string leader_ns = "/" + prefix + std::to_string(declare_parameter<int>("leader_id", 0)) + "/";
    fw_states_sub_ = create_subscription<msg::FwStates>(ns + "fixed_wing_formation_control/fw_states", 10,
        [this](const msg::FwStates::SharedPtr m) { fwstates_ = *m; });
    leader_states_sub_ = create_subscription<msg::LeaderStates>(
        leader_ns + "fixed_wing_formation_control/leader_states", 10,
        [this](const msg::LeaderStates::SharedPtr m) { leaderstates_ = *m; });
    fwmonitor_sub_ = create_subscription<msg::FwMonitor>(ns + "fixed_wing_formation_control/fwmonitor_flags", 10,
        [this](const msg::FwMonitor::SharedPtr m) { fwmonitor_flag_ = *m; });
    fw_cmd_mode_sub_ = create_subscription<msg::FwCmdMode>(ns + "fixed_wing_formation_control/fw_cmd_mode", 10,
        [this](const msg::FwCmdMode::SharedPtr m) { fw_cmd_mode_ = *m; });
    fw_cmd_pub_ = create_publisher<msg::FwCmd>(ns + "fixed_wing_formation_control/fw_cmd", 10);
    formation_control_states_pub_ = create_publisher<msg::FormationControlStates>(
        ns + "fixed_wing_formation_control/formation_control_states", 10);
    fw_current_mode_pub_ = create_publisher<msg::FwCurrentMode>(ns + "fixed_wing_formation_control/fw_current_mode", 10);
    // TECS's time steps and this 50 Hz loop (as ROS 1) in ROS time: simulation time with use_sim_time
    sys_time_source() = [this]() { return static_cast<long>(get_clock()->now().nanoseconds() / 1000000); };
    timer_ = create_timer(std::chrono::milliseconds(20), [this]() { step(); });
  }

private:
  void step() {
    const auto &m = fw_cmd_mode_;
    if (!m.need_take_off && !m.need_formation && !m.need_land && !m.need_idel && m.need_protected) {
      fw_current_mode_.mode = msg::FwCurrentMode::FW_IN_PROTECT;
    } else if (!m.need_take_off && !m.need_formation && !m.need_land && m.need_idel && !m.need_protected) {
      fw_current_mode_.mode = msg::FwCurrentMode::FW_IN_IDEL;
    } else if (!m.need_take_off && !m.need_formation && m.need_land && !m.need_idel && !m.need_protected) {
      fw_current_mode_.mode = msg::FwCurrentMode::FW_IN_LANDING;  // TODO in ROS 1 as well
    } else if (!m.need_take_off && m.need_formation && !m.need_land && !m.need_idel && !m.need_protected) {
      if (plane_id_ != 0) {  // only the followers fly the formation
        formation_type_id_ = m.formation_type;
        control_formation();
      } else {  // the leader: OFFBOARD (pack_fw_states sends its loiter setpoint)
        fw_4cmd_.cmd_mode = "OFFBOARD";
        fw_cmd_pub_->publish(fw_4cmd_);
      }
      fw_current_mode_.mode = msg::FwCurrentMode::FW_IN_FORMATION;
    } else if (m.need_take_off && !m.need_formation && !m.need_land && !m.need_idel && !m.need_protected) {
      fw_current_mode_.mode = msg::FwCurrentMode::FW_IN_TAKEOFF;
    }
    fw_current_mode_.plane_id = plane_id_;
    fw_current_mode_pub_->publish(fw_current_mode_);
  }

  void control_formation() {
    fw_col_mode_current_ = fwstates_.control_mode;
    // leader
    leader_states_.air_speed = leaderstates_.airspeed;
    leader_states_.altitude = leaderstates_.relative_alt;
    leader_states_.latitude = leaderstates_.latitude;
    leader_states_.longitude = leaderstates_.longitude;
    leader_states_.relative_alt = leaderstates_.relative_alt;
    leader_states_.global_vel_x = leaderstates_.global_vel_x;
    leader_states_.global_vel_y = leaderstates_.global_vel_y;
    leader_states_.global_vel_z = leaderstates_.global_vel_z;
    leader_states_.ned_vel_x = leaderstates_.ned_vel_x;
    leader_states_.ned_vel_y = leaderstates_.ned_vel_y;
    leader_states_.ned_vel_z = leaderstates_.ned_vel_z;
    leader_states_.pitch_angle = leaderstates_.pitch_angle;
    leader_states_.roll_angle = leaderstates_.roll_angle;
    leader_states_.yaw_angle = leaderstates_.yaw_angle;
    leader_states_.yaw_valid = false;  // the leader's yaw is not available directly
    leader_states_.yaw_rate = leaderstates_.yaw_rate;
    // this vehicle
    auto &t = thisfw_states_;
    t.flight_mode = fwstates_.control_mode;
    t.air_speed = fwstates_.air_speed;
    t.altitude = fwstates_.relative_alt;
    t.altitude_lock = true;  // for TECS
    t.in_air = true;         // for TECS
    t.latitude = fwstates_.latitude;
    t.longitude = fwstates_.longitude;
    t.relative_alt = fwstates_.relative_alt;
    t.ned_vel_x = fwstates_.ned_vel_x;
    t.ned_vel_y = fwstates_.ned_vel_y;
    t.ned_vel_z = fwstates_.ned_vel_z;
    t.global_vel_x = fwstates_.global_vel_x;
    t.global_vel_y = fwstates_.global_vel_y;
    t.global_vel_z = fwstates_.global_vel_z;
    t.pitch_angle = fwstates_.pitch_angle;
    t.roll_angle = fwstates_.roll_angle;
    t.yaw_angle = fwstates_.yaw_angle;
    t.att_quat[0] = fwstates_.att_quater.w;
    t.att_quat[1] = fwstates_.att_quater.x;
    t.att_quat[2] = fwstates_.att_quater.y;
    t.att_quat[3] = fwstates_.att_quater.z;
    quat_2_rotmax(t.att_quat, t.rotmat);
    t.body_acc[0] = fwstates_.body_acc_x;
    t.body_acc[1] = fwstates_.body_acc_y;
    t.body_acc[2] = fwstates_.body_acc_z;
    matrix_plus_vector_3(t.ned_acc, t.rotmat, t.body_acc);
    t.wind_estimate_x = fwstates_.wind_estimate_x;
    t.wind_estimate_y = fwstates_.wind_estimate_y;
    t.wind_estimate_z = fwstates_.wind_estimate_z;
    t.yaw_rate = fwstates_.yaw_rate;

    formation_controller_.set_fw_planeID(plane_id_);
    formation_controller_.set_formation_type(formation_type_id_);
    if (fw_col_mode_current_ != fw_col_mode_last_) {  // reset on entering the mode
      formation_controller_.reset_formation_controller();
    }
    formation_controller_.update_led_fol_states(&leader_states_, &thisfw_states_);
    formation_controller_.control_law();
    formation_controller_.get_formation_4cmd(formation_cmd_);
    formation_controller_.get_formation_sp(formation_sp_);
    formation_controller_.get_formation_error(formation_error_);

    fw_4cmd_.throttle_sp = formation_cmd_.thrust;
    fw_4cmd_.roll_angle_sp = formation_cmd_.roll;
    fw_4cmd_.pitch_angle_sp = formation_cmd_.pitch;
    // slew limits (not in ROS 1): the lateral law reversed the bank in a second and the followers departed
    const double dt = 0.02;  // the 50 Hz loop
    const auto slew = [dt](double target, double prev, double rate) {
      return std::clamp(target, prev - rate * dt, prev + rate * dt);
    };
    fw_4cmd_.roll_angle_sp = slew(fw_4cmd_.roll_angle_sp, last_roll_sp_, 0.52);    // 30 deg/s
    fw_4cmd_.pitch_angle_sp = slew(fw_4cmd_.pitch_angle_sp, last_pitch_sp_, 0.26);  // 15 deg/s
    // stall guard (not in ROS 1): in OFFBOARD attitude mode PX4's own airspeed protection does not act, and a
    // follower far from its slot climbed in a steep bank until it stalled and spun in
    if (thisfw_states_.air_speed < stall_guard_airspeed_) {
      fw_4cmd_.pitch_angle_sp = std::min(fw_4cmd_.pitch_angle_sp, 0.0);
      fw_4cmd_.roll_angle_sp = std::clamp(fw_4cmd_.roll_angle_sp, -0.35, 0.35);
      fw_4cmd_.throttle_sp = 1.0;
    }
    last_roll_sp_ = fw_4cmd_.roll_angle_sp;
    last_pitch_sp_ = fw_4cmd_.pitch_angle_sp;
    fw_4cmd_.yaw_angle_sp = formation_cmd_.yaw;
    fw_4cmd_.cmd_mode = "OFFBOARD";
    fw_cmd_pub_->publish(fw_4cmd_);
    formation_states_pub();
    fw_col_mode_last_ = fw_col_mode_current_;
  }

  void formation_states_pub() {
    auto &s = formation_control_states_;
    s.plane_id = plane_id_;
    s.err_p_n = formation_error_.P_N;
    s.err_p_e = formation_error_.P_E;
    s.err_p_d = formation_error_.P_D;
    s.err_p_ne = formation_error_.P_NE;
    s.led_fol_vxb = formation_error_.led_fol_vxb;
    s.led_fol_vyb = formation_error_.led_fol_vyb;
    s.led_fol_vzb = formation_error_.led_fol_vzb;
    s.err_pxk = formation_error_.PXk;
    s.err_pyk = formation_error_.PYk;
    s.err_pzk = formation_error_.PZk;
    s.err_vxk = formation_error_.VXk;
    s.err_vyk = formation_error_.VYk;
    s.led_fol_vxk = formation_error_.led_fol_vxk;
    s.led_fol_vyk = formation_error_.led_fol_vyk;
    s.led_fol_vzk = formation_error_.led_fol_vzk;
    s.eta_deg = rad_2_deg(formation_error_.led_fol_eta);
    s.sp_air_speed = formation_sp_.air_speed;
    s.sp_altitude = formation_sp_.altitude;
    s.sp_ground_speed = formation_sp_.ground_speed;
    s.sp_latitude = formation_sp_.latitude;
    s.sp_longitude = formation_sp_.longitude;
    s.sp_ned_vel_x = formation_sp_.ned_vel_x;
    s.sp_ned_vel_y = formation_sp_.ned_vel_y;
    s.sp_ned_vel_z = formation_sp_.ned_vel_z;
    s.sp_relative_alt = formation_sp_.relative_alt;
    formation_control_states_pub_->publish(s);
  }

  int plane_id_;
  int formation_type_id_ = 1;
  double stall_guard_airspeed_ = 10.0;  // m/s (standard_vtol: stall at about 10, trim 15)
  double last_roll_sp_ = 0.0, last_pitch_sp_ = 0.0;  // rad, for the slew limits
  std::string fw_col_mode_current_{"MANUAL"}, fw_col_mode_last_{"MANUAL"};
  msg::FwStates fwstates_;
  msg::FwCmd fw_4cmd_;
  msg::LeaderStates leaderstates_;
  msg::FormationControlStates formation_control_states_;
  msg::FwMonitor fwmonitor_flag_;
  msg::FwCmdMode fw_cmd_mode_;
  msg::FwCurrentMode fw_current_mode_;
  ABS_FORMATION_CONTROLLER formation_controller_;
  ABS_FORMATION_CONTROLLER::_s_leader_states leader_states_;
  ABS_FORMATION_CONTROLLER::_s_fw_states thisfw_states_;
  ABS_FORMATION_CONTROLLER::_s_4cmd formation_cmd_;
  ABS_FORMATION_CONTROLLER::_s_fw_error formation_error_;
  ABS_FORMATION_CONTROLLER::_s_fw_sp formation_sp_;

  rclcpp::Subscription<msg::FwStates>::SharedPtr fw_states_sub_;
  rclcpp::Subscription<msg::LeaderStates>::SharedPtr leader_states_sub_;
  rclcpp::Subscription<msg::FwMonitor>::SharedPtr fwmonitor_sub_;
  rclcpp::Subscription<msg::FwCmdMode>::SharedPtr fw_cmd_mode_sub_;
  rclcpp::Publisher<msg::FwCmd>::SharedPtr fw_cmd_pub_;
  rclcpp::Publisher<msg::FormationControlStates>::SharedPtr formation_control_states_pub_;
  rclcpp::Publisher<msg::FwCurrentMode>::SharedPtr fw_current_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskMain>());
  rclcpp::shutdown();
  return 0;
}

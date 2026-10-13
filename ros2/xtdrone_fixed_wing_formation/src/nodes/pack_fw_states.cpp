// pack_fw_states (chapter 35), ROS 2 port of XTDrone's coordination/fixed_wing_formation_control
// pack_fw_states: gathers the MAVROS topics of one fixed-wing vehicle into FwStates and sends the controller's
// commands to PX4. Followers (plane_id > 0): attitude and throttle (setpoint_raw/attitude) from task_main's
// FwCmd; the leader (plane_id 0): a loiter position setpoint over the UGV (setpoint_raw/local). Both switch
// PX4 to the mode named in FwCmd.cmd_mode.
//   ros2 run xtdrone_fixed_wing_formation pack_fw_states --ros-args -p plane_id:=1
// The ROS 1 version had one executable per plane (pack_leader_states, pack_fw1_states, ...).
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

#include "xtdrone_fixed_wing_formation/msg/fw_cmd.hpp"
#include "xtdrone_fixed_wing_formation/msg/fw_states.hpp"
#include "../fixed_wing_lib/mathlib.hpp"

using xtdrone_fixed_wing_formation::msg::FwCmd;
using xtdrone_fixed_wing_formation::msg::FwStates;

class PackFwStates : public rclcpp::Node {
public:
  PackFwStates() : Node("pack_fw_states") {
    plane_id_ = declare_parameter<int>("plane_id", 1);
    const auto prefix = declare_parameter<std::string>("vehicle_prefix", "standard_vtol_");
    ns_ = "/" + prefix + std::to_string(plane_id_) + "/";
    const auto ugv_topic = declare_parameter<std::string>("ugv_odom_topic", "/ugv_0/ground_truth/odom");
    // The leader loiters over the UGV. Its setpoint is in its local frame (origin at its spawn point): the
    // world position of the UGV minus the leader's spawn position (ROS 1: the VTOL started at x = -1500,
    // hence "+ 1500"; here at -600, see the scenario outdoor2_fw_formation), at loiter_altitude.
    spawn_x_ = declare_parameter<double>("spawn_x", -600.0);
    spawn_y_ = declare_parameter<double>("spawn_y", 0.0);
    loiter_altitude_ = declare_parameter<double>("loiter_altitude", 50.0);
    orbit_radius_ = declare_parameter<double>("orbit_radius", 150.0);  // 0: the UGV position itself, as ROS 1
    lead_angle_ = declare_parameter<double>("orbit_lead_angle", 0.6);   // rad

    auto qos = rclcpp::SensorDataQoS();
    auto sub = [&](auto *member, const std::string &topic) {
      using M = std::remove_pointer_t<decltype(member)>;
      return create_subscription<M>(ns_ + topic, qos, [member](const typename M::SharedPtr m) { *member = *m; });
    };
    subs_.push_back(sub(&state_, "mavros/state"));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(ns_ + "mavros/imu/data", qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr m) {
          imu_ = *m;
          float q[4] = {float(m->orientation.w), float(m->orientation.x), float(m->orientation.y),
                        float(m->orientation.z)};
          quaternion_2_euler(q, att_angle_euler_);
        });
    subs_.push_back(sub(&global_position_, "mavros/global_position/global"));
    subs_.push_back(sub(&rel_alt_, "mavros/global_position/rel_alt"));
    subs_.push_back(sub(&velocity_global_, "mavros/global_position/raw/gps_vel"));
    subs_.push_back(sub(&local_position_, "mavros/local_position/pose"));
    subs_.push_back(sub(&velocity_ned_, "mavros/local_position/velocity_local"));
    subs_.push_back(sub(&acc_ned_, "mavros/local_position/accel"));
    subs_.push_back(sub(&wind_, "mavros/wind_estimation"));
    subs_.push_back(sub(&battery_, "mavros/battery"));
    subs_.push_back(sub(&altitude_, "mavros/altitude"));
    subs_.push_back(sub(&vfr_hud_, "mavros/vfr_hud"));
    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(ugv_topic, qos,  // absolute, not per vehicle
        [this](const nav_msgs::msg::Odometry::SharedPtr m) { ugv_ = *m; }));
    cmd_sub_ = create_subscription<FwCmd>(ns_ + "fixed_wing_formation_control/fw_cmd", 10,
        [this](const FwCmd::SharedPtr m) { cmd_ = *m; });

    local_pos_sp_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(ns_ + "mavros/setpoint_raw/local", 10);
    att_sp_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(ns_ + "mavros/setpoint_raw/attitude", 10);
    states_pub_ = create_publisher<FwStates>(ns_ + "fixed_wing_formation_control/fw_states", 10);
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(ns_ + "mavros/set_mode");

    timer_ = create_timer(std::chrono::milliseconds(20), [this]() { step(); });  // 50 Hz (ROS time), as ROS 1
    RCLCPP_INFO(get_logger(), "plane %d (%s): %s", plane_id_, ns_.c_str(), plane_id_ == 0 ? "leader" : "follower");
  }

private:
  void step() {
    pack_fw_states();
    if (plane_id_ != 0) {
      msg_to_mavros();
    } else {
      leader_fol_ugv();
    }
    srv_to_mavros();
  }

  void pack_fw_states() {
    FwStates &s = fw_states_;
    s.plane_id = plane_id_;
    s.control_mode = state_.mode;
    s.in_air = state_.armed;
    s.altitude = global_position_.altitude;
    s.latitude = global_position_.latitude;
    s.longitude = global_position_.longitude;
    // ENU (MAVROS) -> NED (the controller)
    s.global_vel_x = velocity_global_.twist.linear.y;
    s.global_vel_y = velocity_global_.twist.linear.x;
    s.global_vel_z = -velocity_global_.twist.linear.z;
    s.relative_alt = rel_alt_.data;
    s.roll_angle = att_angle_euler_[0];
    s.pitch_angle = -att_angle_euler_[1];
    s.yaw_angle = -att_angle_euler_[2] + deg_2_rad(90.0);
    if (s.yaw_angle <= 0) {
      s.yaw_angle += deg_2_rad(360.0);
    }
    float att_angle[3] = {float(s.roll_angle), float(s.pitch_angle), float(s.yaw_angle)};
    float att_quat[4];
    euler_2_quaternion(att_angle, att_quat);
    s.att_quater.w = att_quat[0];
    s.att_quater.x = att_quat[1];
    s.att_quater.y = att_quat[2];
    s.att_quater.z = att_quat[3];
    s.ned_pos_x = local_position_.pose.position.y;
    s.ned_pos_y = local_position_.pose.position.x;
    s.ned_pos_z = -local_position_.pose.position.z;
    s.ned_vel_x = velocity_ned_.twist.linear.y;
    s.ned_vel_y = velocity_ned_.twist.linear.x;
    s.ned_vel_z = -velocity_ned_.twist.linear.z;
    s.body_acc_x = imu_.linear_acceleration.x;
    s.body_acc_y = imu_.linear_acceleration.y;
    s.body_acc_z = imu_.linear_acceleration.z;
    s.body_acc.x = imu_.linear_acceleration.x;
    s.body_acc.y = imu_.linear_acceleration.y;
    s.body_acc.z = imu_.linear_acceleration.z;
    s.relative_hight = altitude_.relative;
    s.ned_altitude = altitude_.local;
    // without a valid airspeed (FW_USE_AIRSPD=0, or the airspeed selector down) PX4 reports 0: take the ground
    // speed, else the formation controller's TECS sees a stall and dives
    s.air_speed = std::isfinite(vfr_hud_.airspeed) && vfr_hud_.airspeed > 0.5f ? vfr_hud_.airspeed : vfr_hud_.groundspeed;
    s.ground_speed = vfr_hud_.groundspeed;
    s.wind_estimate_x = wind_.twist.twist.linear.y;
    s.wind_estimate_y = wind_.twist.twist.linear.x;
    s.wind_estimate_z = -wind_.twist.twist.linear.z;
    s.battery_current = battery_.current;
    s.battery_precentage = battery_.percentage;
    s.battery_voltage = battery_.voltage;
    s.pitch_rate = -imu_.angular_velocity.y;
    s.roll_rate = imu_.angular_velocity.x;
    s.yaw_rate = -imu_.angular_velocity.z;
    states_pub_->publish(s);
  }

  // follower: the controller's attitude and throttle
  void msg_to_mavros() {
    float angle[3] = {float(cmd_.roll_angle_sp), float(-cmd_.pitch_angle_sp),
                      float(-cmd_.yaw_angle_sp + deg_2_rad(90.0))};
    float quat[4];
    euler_2_quaternion(angle, quat);
    mavros_msgs::msg::AttitudeTarget att_sp;
    att_sp.header.stamp = now();
    att_sp.type_mask = 7;  // ignore the body rates
    att_sp.orientation.w = quat[0];
    att_sp.orientation.x = quat[1];
    att_sp.orientation.y = quat[2];
    att_sp.orientation.z = quat[3];
    att_sp.thrust = cmd_.throttle_sp;
    att_sp_pub_->publish(att_sp);
  }

  // leader: loiter over the UGV
  void leader_fol_ugv() {
    mavros_msgs::msg::PositionTarget sp;
    sp.header.stamp = now();
    // the UGV in the leader's local frame (ENU, origin at the leader's spawn point)
    const double ux = ugv_.pose.pose.position.x - spawn_x_, uy = ugv_.pose.pose.position.y - spawn_y_;
    sp.position.x = ux;
    sp.position.y = uy;
    sp.position.z = loiter_altitude_;
    // ROS 1 sent the UGV position with type_mask 12288, PX4's old "loiter" bits, and PX4 loitered around it. PX4
    // 1.16 ignores those bits; a position-only setpoint flies to the point and keeps crossing it in ~25 m turns,
    // too tight for the followers' fixed offsets. Within orbit_radius of the UGV the setpoint is therefore a
    // point on a circle of that radius around it, lead_angle ahead of the leader (counter-clockwise).
    const double dx = local_position_.pose.position.x - ux, dy = local_position_.pose.position.y - uy;
    const double dist = std::hypot(dx, dy);
    if (orbit_radius_ > 0.0 && dist < 2.0 * orbit_radius_) {
      const double ahead = std::atan2(dy, dx) + lead_angle_;
      sp.position.x = ux + orbit_radius_ * std::cos(ahead);
      sp.position.y = uy + orbit_radius_ * std::sin(ahead);
    }
    using PT = mavros_msgs::msg::PositionTarget;
    sp.type_mask = PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ | PT::IGNORE_AFX | PT::IGNORE_AFY |
                   PT::IGNORE_AFZ | PT::IGNORE_YAW | PT::IGNORE_YAW_RATE;
    sp.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;  // MAVROS converts from ENU
    local_pos_sp_pub_->publish(sp);
  }

  // PX4 mode requested by the controller. ROS 1 called the (blocking) service in every 50 Hz cycle; here only
  // when the mode differs, at most once a second.
  void srv_to_mavros() {
    const std::string &mode = cmd_.cmd_mode;
    if (mode.empty() || mode == state_.mode || !set_mode_client_->service_is_ready()) {
      return;
    }
    if ((now() - last_mode_request_).seconds() < 1.0) {
      return;
    }
    last_mode_request_ = now();
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    set_mode_client_->async_send_request(req);
    RCLCPP_INFO(get_logger(), "set mode %s", mode.c_str());
  }

  int plane_id_;
  std::string ns_;
  double spawn_x_, spawn_y_, loiter_altitude_, orbit_radius_, lead_angle_;
  float att_angle_euler_[3] = {0, 0, 0};
  rclcpp::Time last_mode_request_{0, 0, RCL_ROS_TIME};

  mavros_msgs::msg::State state_;
  sensor_msgs::msg::Imu imu_;
  sensor_msgs::msg::NavSatFix global_position_;
  std_msgs::msg::Float64 rel_alt_;
  geometry_msgs::msg::TwistStamped velocity_global_, velocity_ned_;
  geometry_msgs::msg::PoseStamped local_position_;
  geometry_msgs::msg::AccelWithCovarianceStamped acc_ned_;
  geometry_msgs::msg::TwistWithCovarianceStamped wind_;
  sensor_msgs::msg::BatteryState battery_;
  mavros_msgs::msg::Altitude altitude_;
  mavros_msgs::msg::VfrHud vfr_hud_;
  nav_msgs::msg::Odometry ugv_;
  FwCmd cmd_;
  FwStates fw_states_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<FwCmd>::SharedPtr cmd_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_pos_sp_pub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_sp_pub_;
  rclcpp::Publisher<FwStates>::SharedPtr states_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PackFwStates>());
  rclcpp::shutdown();
  return 0;
}

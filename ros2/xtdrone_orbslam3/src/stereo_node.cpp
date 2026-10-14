// XTDrone stereo ORB-SLAM3 node for ROS 2 (chapter 13). Port of
// XTDrone's sensing/slam/vslam/ORB_SLAM3/Examples/ROS/ORB_SLAM3/src/ros_stereo.cc (XTDrone version):
// tracks the stereo pair and publishes the camera pose as /orbslam3/vision_pose/pose in a FLU world
// frame (x forward, y left, z up at the start), ready for /<vehicle>/mavros/vision_pose/pose.
//
//   ros2 run xtdrone_orbslam3 stereo_node <ORBvoc.txt> <settings.yaml> [use_viewer true|false]
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "System.h"

using ImageMsg = sensor_msgs::msg::Image;
// exact stamps: under load gz drops left and right frames independently, and a pair from two
// different render steps breaks the stereo matching while the vehicle moves
using SyncPolicy = message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg>;

class StereoNode : public rclcpp::Node
{
public:
  StereoNode(const std::string & voc, const std::string & settings, bool viewer)
  : Node("orbslam3_stereo")
  {
    slam_ = std::make_unique<ORB_SLAM3::System>(voc, settings, ORB_SLAM3::System::STEREO, viewer);
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/vision_pose/pose", 10);
    auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    left_.subscribe(this, "/camera/left/image_raw", qos);
    right_.subscribe(this, "/camera/right/image_raw", qos);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), left_, right_);
    sync_->registerCallback(std::bind(&StereoNode::grab, this, std::placeholders::_1, std::placeholders::_2));
    // optical frame (x right, y down, z forward) -> FLU (x forward, y left, z up)
    flu_from_optical_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  }

  ~StereoNode() override { slam_->Shutdown(); }

private:
  void grab(const ImageMsg::ConstSharedPtr & left, const ImageMsg::ConstSharedPtr & right)
  {
    cv_bridge::CvImageConstPtr l, r;
    try {
      l = cv_bridge::toCvShare(left);
      r = cv_bridge::toCvShare(right);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    const double t = rclcpp::Time(left->header.stamp).seconds();
    Sophus::SE3f Tcw = slam_->TrackStereo(l->image, r->image, t);
    if (slam_->GetTrackingState() != ORB_SLAM3::Tracking::OK) {
      return;
    }
    const Sophus::SE3f Twc = Tcw.inverse();
    // world and camera frames are optical frames; express both in FLU
    const Eigen::Matrix3f R = flu_from_optical_ * Twc.rotationMatrix() * flu_from_optical_.transpose();
    const Eigen::Vector3f p = flu_from_optical_ * Twc.translation();
    const Eigen::Quaternionf q(R);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = left->header.stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pub_->publish(pose);
  }

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  message_filters::Subscriber<ImageMsg> left_, right_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  Eigen::Matrix3f flu_from_optical_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() < 3) {
    fprintf(stderr, "usage: stereo_node <ORBvoc.txt> <settings.yaml> [use_viewer]\n");
    return 1;
  }
  const bool viewer = args.size() > 3 && args[3] == "true";
  rclcpp::spin(std::make_shared<StereoNode>(args[1], args[2], viewer));
  rclcpp::shutdown();
  return 0;
}

// grasping_demo (chapter 33), ROS 2 port of XTDrone's sitl_config/robotic_arm/le_arm_grasping: the le_arm under the iris
// picks up the green cube of the grasping world.
//   0. wait until the controllers are active (they come up after move_group);
//   1. wait until vision_manager sees the object;
//   2. MoveIt (group "arm"): the pre-grasp pose behind the object, then the grasp pose around it;
//   3. attach the object (DetachableJoint, ROS 1: gazebo_grasp_fix) and close the gripper (group "gripper");
//   4. lift the object to the carry pose.
// Like ROS 1, the poses are joint targets (ROS 1 hard-coded its grasp pose as well; its camera-to-robot
// conversion was not used). The progress goes to le_arm/grasp_state: detecting, approaching, grasping,
// lifting, grasped (or failed).
//   ros2 run xtdrone_robotic_arm grasping_demo --ros-args -r __ns:=/iris_0
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class GraspingDemo {
public:
  explicit GraspingDemo(const rclcpp::Node::SharedPtr &node) : node_(node) {
    // joint targets of shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2 (radians)
    // (from the arm's kinematics, for the cube of the grasping world 0.2 m ahead of base_link with the iris at
    // x = 0.1: pre-grasp 6 cm behind the cube, grasp around it, carry 2.5 cm higher)
    pregrasp_ = node_->declare_parameter<std::vector<double>>("pregrasp", {0.0, -0.55, 0.3, -0.8, 0.0});
    grasp_ = node_->declare_parameter<std::vector<double>>("grasp", {0.0, -1.15, 0.85, -0.05, 0.0});
    carry_ = node_->declare_parameter<std::vector<double>>("carry", {0.0, -0.95, 0.65, -0.45, 0.0});
    gripper_open_ = node_->declare_parameter<double>("gripper_open", 0.5);
    gripper_closed_ = node_->declare_parameter<double>("gripper_closed", -0.25);
    detect_timeout_ = node_->declare_parameter<double>("detect_timeout", 60.0);

    rclcpp::QoS latched(1);
    latched.transient_local();
    state_pub_ = node_->create_publisher<std_msgs::msg::String>("le_arm/grasp_state", latched);
    attach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("le_arm/gripper/attach", 10);
    detach_pub_ = node_->create_publisher<std_msgs::msg::Empty>("le_arm/gripper/detach", 10);
    object_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>("le_arm/object", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr m) { object_ = *m; seen_ = true; });
  }

  bool run() {
    // The DetachableJoint holds the object from the start (Gazebo Harmonic): release it first.
    for (int i = 0; i < 5; ++i) {
      detach_pub_->publish(std_msgs::msg::Empty());
      std::this_thread::sleep_for(200ms);
    }
    // move_group runs in the vehicle's namespace; MoveGroupInterface ignores the node's namespace and looks for
    // /move_action unless told
    const std::string ns = node_->get_namespace();
    arm_ = std::make_shared<MoveGroupInterface>(node_, MoveGroupInterface::Options("arm", "robot_description", ns));
    gripper_ = std::make_shared<MoveGroupInterface>(
        node_, MoveGroupInterface::Options("gripper", "robot_description", ns));
    for (auto &g : {arm_, gripper_}) {
      g->setMaxVelocityScalingFactor(0.3);
      g->setMaxAccelerationScalingFactor(0.3);
      g->setPlanningTime(5.0);
    }

    state("detecting");
    const auto start = node_->now();
    while (rclcpp::ok() && !seen_ && (node_->now() - start).seconds() < detect_timeout_) {
      std::this_thread::sleep_for(100ms);
    }
    if (!seen_) {
      return fail("the object is not in view of the camera");
    }
    RCLCPP_INFO(node_->get_logger(), "object at pixel (%.0f, %.0f)", object_.point.x, object_.point.y);

    state("approaching");
    if (!move_gripper(gripper_open_) || !move_arm(pregrasp_, "pre-grasp") || !move_arm(grasp_, "grasp")) {
      return fail("approach");
    }
    state("grasping");
    for (int i = 0; i < 5; ++i) {
      attach_pub_->publish(std_msgs::msg::Empty());
      std::this_thread::sleep_for(100ms);
    }
    if (!move_gripper(gripper_closed_)) {
      return fail("closing the gripper");
    }
    state("lifting");
    if (!move_arm(carry_, "carry")) {
      return fail("lifting");
    }
    state("grasped");
    return true;
  }

private:
  bool move_arm(const std::vector<double> &joints, const std::string &what) {
    RCLCPP_INFO(node_->get_logger(), "arm to the %s pose", what.c_str());
    arm_->setJointValueTarget(joints);
    return arm_->move() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  bool move_gripper(double knuckle) {
    // the gripper group has one active joint, left_knuckle_joint; the others mimic it
    gripper_->setJointValueTarget("left_knuckle_joint", knuckle);
    return gripper_->move() == moveit::core::MoveItErrorCode::SUCCESS;
  }

  void state(const std::string &s) {
    std_msgs::msg::String m;
    m.data = s;
    state_pub_->publish(m);
    RCLCPP_INFO(node_->get_logger(), "grasp state: %s", s.c_str());
  }

  bool fail(const std::string &why) {
    RCLCPP_ERROR(node_->get_logger(), "failed: %s", why.c_str());
    state("failed");
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<double> pregrasp_, grasp_, carry_;
  double gripper_open_, gripper_closed_, detect_timeout_;
  std::shared_ptr<MoveGroupInterface> arm_, gripper_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach_pub_, detach_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_sub_;
  geometry_msgs::msg::PointStamped object_;
  bool seen_ = false;
};

// The controllers come up after move_group (spawners) and MoveIt does not wait for them. Called before the
// executor thread starts, on a node of its own.
static bool wait_for_controllers(const std::string &ns, double timeout) {
  auto node = std::make_shared<rclcpp::Node>("grasping_demo_wait", ns);
  auto client = node->create_client<controller_manager_msgs::srv::ListControllers>("controller_manager/list_controllers");
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && std::chrono::steady_clock::now() - start < std::chrono::duration<double>(timeout)) {
    if (client->wait_for_service(1s)) {
      auto fut = client->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
      if (rclcpp::spin_until_future_complete(node, fut, 5s) == rclcpp::FutureReturnCode::SUCCESS) {
        int active = 0;
        const auto response = fut.get();  // kept: a range-for over fut.get()->controller outlives the temporary
        for (const auto &c : response->controller) {
          active += c.state == "active" && (c.name == "le_arm_controller" || c.name == "gripper_controller" ||
                                            c.name == "joint_state_broadcaster");
        }
        if (active == 3) {
          return true;
        }
      }
    }
    std::this_thread::sleep_for(1s);
  }
  return false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "grasping_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
  if (!wait_for_controllers(node->get_namespace(), 180.0)) {
    RCLCPP_ERROR(node->get_logger(), "the arm controllers are not active");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });
  GraspingDemo demo(node);
  const bool ok = demo.run();
  // keep the latched state for the test and the gripper's DetachableJoint as it is
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(500ms);
  }
  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}

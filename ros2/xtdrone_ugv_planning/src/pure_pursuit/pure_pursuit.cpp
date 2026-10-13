// Pure pursuit path follower for the CAT Vehicle UGV (chapter 31), ROS 2 port of XTDrone's pure_pursuit.
//   ros2 run xtdrone_ugv_planning pure_pursuit_node --ros-args -p use_sim_time:=true
// Follows the hybrid A* path (path_topic, /front_end_path) with the vehicle pose from TF map -> base_frame
// and sends speed and front steering angle on cmd_topic (/ugv_0/cmd_vel, linear.x and angular.z: the
// XTDrone UGV interface of chapter 11).
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

#define PI 3.1415926535

std::mutex global_mtx_;
Eigen::VectorXf start_point(6);
Eigen::VectorXf robot_localization(7);  // x y yaw liner_v angle_v liner_acc angle_acc
std::vector<Eigen::Vector3f> Trajectory;

rclcpp::Node::SharedPtr node_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::string world_frame_ = "map";
std::string base_frame_ = "ugv_0/base_link";
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

bool get_new_trajectory = false;
bool end_flag = false;
bool get_localization = false;

/********************************************pure pursuit*********************************************/
float pure_pursuit_k = 0.1;         
float pure_pursuit_Lfc = 2;         
float pure_pursuit_Kp = 1.0;        
float dt = 0.1;                     
float pure_pursuit_L = 2.7;         
float pure_pursuit_liner_v = 5;
float pure_pursuit_min_v = 0.5;   
/*****************************************************************************************************/

float get_yaw(const geometry_msgs::msg::Quaternion& q){
    return float(tf2::getYaw(q));
}

float calculation_yaw_difference(float now_yaw, float last_yaw){
    if(now_yaw - last_yaw > PI){
        return (2 * PI - now_yaw + last_yaw);
    }
    else if( now_yaw - last_yaw < -PI){
        return (2 * PI +now_yaw - last_yaw);
    }
    else{
        return (now_yaw - last_yaw);
    }
}

Eigen::Vector2f calculation_velocity(const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose, float delta_time){
    float liner_v = (now_pose.block<2,1>(0,0) - last_pose.block<2,1>(0,0)).norm() / delta_time;
    float angle_v = calculation_yaw_difference(now_pose(2), last_pose(2)) / delta_time;
    if(abs(liner_v) < 0.001)
        liner_v = 0.0;
    if(abs(angle_v) < 0.001)
        angle_v = 0.0;
    return Eigen::Vector2f(liner_v, angle_v);
}

Eigen::Vector2f calculation_acceleration(const Eigen::Vector2f& last_v, const Eigen::Vector2f& now_v, const Eigen::Vector3f& last_pose, const Eigen::Vector3f& now_pose){
    float x = (now_pose.block<2,1>(0,0) - last_pose.block<2,1>(0,0)).norm();
    float yaw = calculation_yaw_difference(now_pose(2), last_pose(2));
    float acc_x = (now_v(0) * now_v(0) - last_v(0) * last_v(0)) / (2.0 * x);
    float acc_a = (now_v(1) * now_v(1) - last_v(1) * last_v(1)) / (2.0 * yaw);
    if(abs(x) < 0.001)
        acc_x = 0.0;
    if(abs(yaw) < 0.001)
        acc_a = 0.0;
    return Eigen::Vector2f(acc_x, acc_a);
}

void odom_callback(){
    geometry_msgs::msg::TransformStamped transform;
    try{
        transform = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
    }
    catch(const tf2::TransformException &ex){
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "%s", ex.what());
        return;
    }
    const auto& tr = transform.transform.translation;
    const auto& rot = transform.transform.rotation;

    Eigen::Vector3f now_T(tr.x, tr.y, get_yaw(rot));
    static bool first_cal = true;
    static Eigen::Vector3f history_T = now_T;
    static Eigen::Vector2f history_velocity = Eigen::Vector2f::Zero();
    static rclcpp::Time t0 = node_->now();
    rclcpp::Time t1 = node_->now();

    global_mtx_.lock();
    start_point(0) = tr.x;
    start_point(1) = tr.y;
    start_point(2) = rot.w;
    start_point(3) = rot.x;
    start_point(4) = rot.y;
    start_point(5) = rot.z;
    if(first_cal){
        first_cal = false;
        global_mtx_.unlock();
        return;
    }
    auto robot_velocity = calculation_velocity(history_T, now_T, (t1 - t0).seconds());
    auto robot_acceleration = calculation_acceleration(history_velocity, robot_velocity, history_T, now_T);
    robot_localization << now_T(0), now_T(1), now_T(2),
                          robot_velocity(0), robot_velocity(1),
                          robot_acceleration(0), robot_acceleration(1);
    global_mtx_.unlock();
    get_localization = true;
    history_velocity = robot_velocity;
    history_T = now_T;
    t0 = t1;
}

void path_callback(const nav_msgs::msg::Path::SharedPtr msg){

    if (msg->poses.size() < 2)
        return;
    global_mtx_.lock();
    Trajectory.clear();
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = world_frame_;
    path_msg.header.stamp = node_->now();
    path_publisher_->publish(path_msg);
    for(size_t num = 0; num < msg->poses.size(); num++){
        geometry_msgs::msg::PoseStamped poses;
        poses.pose.position.x  = msg->poses[num].pose.position.x;
        poses.pose.position.y  = msg->poses[num].pose.position.y;
        if(num == 0)
            poses.pose.orientation = msg->poses[num + 1].pose.orientation;
        else
            poses.pose.orientation = msg->poses[num].pose.orientation;
        path_msg.poses.push_back(poses);

        Trajectory.push_back(Eigen::Vector3f(msg->poses[num].pose.position.x,
                                             msg->poses[num].pose.position.y,
                                             get_yaw(msg->poses[num].pose.orientation)));
    }
    path_publisher_->publish(path_msg);
    // /front_end_path runs from the vehicle to the goal: no reversal (the ROS 1 follower reversed the path it
    // got on /new_path)
    global_mtx_.unlock();
    get_new_trajectory = true;

}

int calculation_current_index(){
    global_mtx_.lock();
    Eigen::Vector3f current_state = robot_localization.block<3,1>(0,0);
    int index = 0;
    float diff_distance_ = 1000000.0;
    for(size_t index_ = 0; index_ < Trajectory.size(); index_++){
        float temp_diff_distance_ = (current_state.block<2,1>(0,0) - Trajectory[index_].block<2,1>(0,0)).norm();
        if(temp_diff_distance_ <= diff_distance_){
            diff_distance_ = temp_diff_distance_;
            index = index_;
        }
    }

    float  Lf = pure_pursuit_k * robot_localization(3) + pure_pursuit_Lfc;  // 前视距离
    float L = 0.0;
    while(Lf > L && (index+1) < Trajectory.size()){
        Eigen::Vector2f diff_pts = Trajectory[index+1].block<2,1>(0,0) - Trajectory[index].block<2,1>(0,0);
        L += diff_pts.norm();
        index++;
    }
    global_mtx_.unlock();
    return index;
}

float calculation_control_liner_acceleration(float current_velocity){
    global_mtx_.lock();
    float acc = pure_pursuit_Kp * (pure_pursuit_liner_v - current_velocity);
    global_mtx_.unlock();
    return acc;
}

float calculation_control_angle_acceleration(int& current_index){
    int index = calculation_current_index();
    if(current_index >= index)
        index = current_index;
    Eigen::Vector3f current_point;
    if(index < Trajectory.size())
        current_point = Trajectory[index];
    else{
        current_point = Trajectory.back();
        index = Trajectory.size() - 1;
    }
    auto diff_P = current_point - robot_localization.block<3,1>(0,0);
    float alpha = std::atan2(diff_P.y(), diff_P.x()) - robot_localization(2);

    if(robot_localization(3) < 0){
        alpha = PI - alpha;
    }
    float Lf = pure_pursuit_k * robot_localization(3) + pure_pursuit_Lfc;  
    float delta = std::atan2(2.0 * pure_pursuit_L * sin(alpha) / Lf, 1.0);
    current_index = index;
    static float delta_ = 0.0;
    float return_delta_ = (delta - delta_);
    delta_ = delta;
    return delta;
}

void cmd_publish(float acc_liner, float acc_angle, float dt){
    global_mtx_.lock();
    float v_l = robot_localization(3) + acc_liner * dt;
    // at least min_speed while tracking: from standstill (measured speed 0) the ramp above commands only
    // Kp * v * dt (about 0.15 m/s with the simulation's time steps), too little to move the CAT Vehicle,
    // and the follower stayed there for ever
    v_l = std::max(v_l, pure_pursuit_min_v);
//    float v_a = robot_localization(4) + acc_angle * dt;
    // the pure-pursuit steering angle itself (the CAT Vehicle takes the front steering angle in angular.z). The
    // ROS 1 node sent it minus the yaw rate estimated from TF differences (its plain variant is commented out
    // there); with the simulation's small, uneven time steps that estimate cancelled much of the steering and the
    // UGV drifted metres off the path.
    float v_a = acc_angle;
    // float v_a = acc_angle;
    geometry_msgs::msg::Twist vel_cmd;
    vel_cmd.linear.x = v_l;
    vel_cmd.angular.z = v_a;
    global_mtx_.unlock();
    std::cout << "\e[1;33;49m >>> v_l ：" << v_l << " v_a : "<< v_a <<" \e[0m"<< std::endl;
//    std::cout << "\e[1;33;49m >>> acc_liner ：" << acc_liner << " \e[0m" << std::endl;
    vel_publisher_->publish(vel_cmd);
}

void zero_velocity(){
    geometry_msgs::msg::Twist vel_cmd;
    vel_cmd.linear.x = 0.0;
    vel_cmd.angular.z = 0.0;
    vel_publisher_->publish(vel_cmd);
}

bool arrived_goal(){
    std::cout << "\e[1;33;49m >>> to the goal" << (Trajectory.back().block<2,1>(0,0) - robot_localization.block<2,1>(0,0)).norm() << "m \e[0m"<< std::endl;
    if((Trajectory.back().block<2,1>(0,0) - robot_localization.block<2,1>(0,0)).norm() < 1.0) {
        std::cout << "\e[1;33;49m >>> reached! \e[0m"<< std::endl;
        return true;
    }
    else
        return false;
}

void pure_pursuit(){
    while(true){
        std::cout << "\e[1;33;49m >>> wait for the path\e[0m"<< std::endl;
        while(true) {
            if(end_flag)
                break;
            if (get_new_trajectory && get_localization) {
                std::cout << "\e[1;33;49m >>> new path! \e[0m"<< std::endl;
                get_new_trajectory = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "\e[1;33;49m >>> tracking \e[0m"<< std::endl;
        int index = calculation_current_index();
        rclcpp::Time t_0 = node_->now();
        std::cout << "\e[1;33;49m >>> Trajectory.size() : "<< Trajectory.size() << "\e[0m"<< std::endl;
        std::cout << "\e[1;33;49m >>> index : "<< index << "\e[0m"<< std::endl;
        if(Trajectory.size() == 0)
            continue;
        // until the vehicle is within 1 m of the last point (the ROS 1 loop ended when the look-ahead point
        // reached the end of the path, a look-ahead distance short of the goal)
        while(true){
            if(arrived_goal()) {
                zero_velocity();
                break;
            }
            if(get_new_trajectory)
                break;
            index = calculation_current_index();
            std::cout << "\e[1;33;49m >>> index : "<< index << "\e[0m"<< std::endl;
            std::cout << "\e[1;33;49m >>> Trajectory(index) : "<< Trajectory[index].transpose() << "\e[0m"<< std::endl;
            std::cout << "\e[1;33;49m >>> robot_localization : "<< robot_localization.transpose() << "\e[0m"<< std::endl;
            float acc_liner = calculation_control_liner_acceleration(robot_localization(3));
            float acc_angle = calculation_control_angle_acceleration(index);
            rclcpp::Time t_1 = node_->now();
            float delta_t = (t_1 - t_0).seconds();
            cmd_publish(acc_liner, acc_angle, delta_t);
            t_0 = t_1;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        zero_velocity();

        if(end_flag)
            break;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node_ = std::make_shared<rclcpp::Node>("pure_pursuit");
    const auto path_topic = node_->declare_parameter<std::string>("path_topic", "/front_end_path");
    const auto cmd_topic = node_->declare_parameter<std::string>("cmd_topic", "/ugv_0/cmd_vel");
    base_frame_ = node_->declare_parameter<std::string>("base_frame", base_frame_);
    world_frame_ = node_->declare_parameter<std::string>("world_frame", world_frame_);
    pure_pursuit_liner_v = node_->declare_parameter<double>("speed", pure_pursuit_liner_v);
    pure_pursuit_L = node_->declare_parameter<double>("wheel_base", pure_pursuit_L);
    pure_pursuit_min_v = node_->declare_parameter<double>("min_speed", pure_pursuit_min_v);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // the planner latches its path (transient local)
    auto path_sub = node_->create_subscription<nav_msgs::msg::Path>(
        path_topic, rclcpp::QoS(1).transient_local().reliable(), &path_callback);
    vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 1);
    path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/visual_path", rclcpp::QoS(1).transient_local());
    auto odom_timer = node_->create_wall_timer(std::chrono::milliseconds(100), &odom_callback);

    std::thread pure_pursuit_thread(pure_pursuit);
    pure_pursuit_thread.detach();

    rclcpp::spin(node_);
    end_flag = true;
    zero_velocity();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    rclcpp::shutdown();
    return 0;
}

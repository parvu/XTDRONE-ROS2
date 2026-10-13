// vision_manager (chapter 33), ROS 2 port of sitl_config/robotic_arm/le_arm_vision: finds the green object in the
// image of the arm camera. ROS 1 thresholded the green channel (> 200) and took the centre of the bounding box of
// the pixels; here green must clearly exceed red and blue (the cube is in shade; white sky and ground are not it).
// Publishes the object's pixel position (x, y) and its size in pixels (z) on le_arm/object, and nothing while the
// object is not in view.
//   ros2 run xtdrone_robotic_arm vision_manager --ros-args -r __ns:=/iris_0
#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class VisionManager : public rclcpp::Node {
public:
  VisionManager() : Node("vision_manager") {
    min_pixels_ = declare_parameter<int>("min_pixels", 20);
    pub_ = create_publisher<geometry_msgs::msg::PointStamped>("le_arm/object", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>("le_arm/camera/image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { image_cb(msg); });
  }

private:
  void image_cb(const sensor_msgs::msg::Image::SharedPtr &msg) {
    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    cv::Mat denoised, mask;
    cv::medianBlur(img, denoised, 3);
    // green clearly above red and blue (the cube is a dark green, about (0, 100, 0), in the shade under the iris)
    std::vector<cv::Mat> bgr;
    cv::split(denoised, bgr);
    cv::Mat g = bgr[1];
    mask = (g > 60) & (g > bgr[2] * 1.5 + 20) & (g > bgr[0] * 1.5 + 20);
    cv::dilate(mask, mask, cv::Mat());
    std::vector<cv::Point> points;
    cv::findNonZero(mask, points);
    if (static_cast<int>(points.size()) < min_pixels_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "no object in view");
      return;
    }
    const cv::Rect box = cv::boundingRect(points);
    geometry_msgs::msg::PointStamped p;
    p.header = msg->header;
    p.point.x = box.x + box.width / 2.0;
    p.point.y = box.y + box.height / 2.0;
    p.point.z = static_cast<double>(points.size());
    pub_->publish(p);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "object at pixel (%.0f, %.0f), %zu pixels",
                         p.point.x, p.point.y, points.size());
  }

  int min_pixels_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionManager>());
  rclcpp::shutdown();
  return 0;
}

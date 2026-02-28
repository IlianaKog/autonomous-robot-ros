#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class InspectionBehaviorNode : public rclcpp::Node
{
public:
  InspectionBehaviorNode() : Node("inspection_behavior_node"), target_detected_(false)
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&InspectionBehaviorNode::image_callback, this, _1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_perception", 10);
    
    RCLCPP_INFO(this->get_logger(), "Perception node started. Waiting for red targets...");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (target_detected_) return; // Only process once per target or cooldown
    
    try {
      cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      cv::Mat hsv_frame, mask;
      
      cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
      
      // OpenCV HSV red ranges (0-10 and 160-180)
      cv::Mat mask1, mask2;
      cv::inRange(hsv_frame, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
      cv::inRange(hsv_frame, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
      mask = mask1 | mask2;

      // Morphological operations to reduce noise
      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
      cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 20000.0) { // Large red blob detected at close range!
          RCLCPP_WARN(this->get_logger(), "RED TARGET DETECTED! Area: %.2f", area);
          handle_target_detection(frame);
          break; // Stop checking further contours
        }
      }

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void handle_target_detection(const cv::Mat& frame)
  {
    target_detected_ = true;
    
    // 1. Force halt the robot
    auto twist = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist); // Sending all 0s to halt

    // 2. Capture and save the image
    std::string filename = "/tmp/inspection_capture_" + std::to_string(this->now().nanoseconds()) + ".jpg";
    cv::imwrite(filename, frame);
    RCLCPP_INFO(this->get_logger(), "Target Image saved to: %s", filename.c_str());

    // 3. Cooldown / resume logic (simulate inspection time)
    RCLCPP_INFO(this->get_logger(), "Inspecting object for 5 seconds...");
    
    timer_ = this->create_wall_timer(5000ms, [this]() {
      RCLCPP_INFO(this->get_logger(), "Inspection complete. Resuming mission.");
      target_detected_ = false;
      timer_->cancel();
    });
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool target_detected_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InspectionBehaviorNode>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

using namespace std::chrono_literals;

class WaypointFollower : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  WaypointFollower() : Node("waypoint_follower"), current_wp_index_(0)
  {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // X, Y map coordinates (meters)
    waypoints_ = {
      {2.0, 0.0},
      {2.0, 2.0},
      {3.7, 3.0}
    };

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&WaypointFollower::start_mission, this));
  }

private:
  void start_mission()
  {
    timer_->cancel(); // Run once
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "navigate_to_pose action server not available. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Starting Waypoint mission!");
    send_next_waypoint();
  }

  void send_next_waypoint()
  {
    if (current_wp_index_ >= waypoints_.size()) {
      RCLCPP_INFO(this->get_logger(), "MISSION COMPLETE. All waypoints reached.");
      return;
    }

    auto wp = waypoints_[current_wp_index_];
    RCLCPP_INFO(this->get_logger(), "Navigating to Map Pos: x=%.2f, y=%.2f", wp.first, wp.second);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = wp.first;
    goal_msg.pose.pose.position.y = wp.second;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&WaypointFollower::result_callback, this, std::placeholders::_1);

    nav_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void result_callback(const GoalHandleNav::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached!", current_wp_index_ + 1);
      current_wp_index_++;
      send_next_waypoint();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint. Mission aborted.");
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::pair<double, double>> waypoints_;
  size_t current_wp_index_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointFollower>());
  rclcpp::shutdown();
  return 0;
}

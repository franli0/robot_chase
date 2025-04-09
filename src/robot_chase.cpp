#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class RobotChase : public rclcpp::Node
{
public:
  RobotChase()
  : Node("robot_chase"),
    initialized_(false),
    morty_is_moving_(false),
    wait_for_tf_(true),
    tf_ready_(false)
  {
    // Create a transform listener with a longer cache time
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), 10s);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher for Rick's velocity commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

    // Create a timer to periodically check the transform and update velocity
    // Start with a longer interval to give TF time to populate
    timer_ = this->create_wall_timer(
      500ms, std::bind(&RobotChase::timer_callback, this));

    // PID controller gains - these can be tuned for better performance
    kp_distance_ = 0.3;  // Proportional gain for distance
    kp_yaw_ = 1.0;       // Proportional gain for yaw
    
    // Safety parameters
    min_distance_ = 0.4;  // Minimum safe distance to maintain (increased for better behavior)
    max_linear_velocity_ = 0.5;  // Maximum linear velocity
    max_angular_velocity_ = 1.0;  // Maximum angular velocity
    
    // Movement detection parameters
    movement_threshold_ = 0.1;  // Distance Morty must move to trigger chase
    last_morty_x_ = 0.0;
    last_morty_y_ = 0.0;
    last_position_update_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Robot Chase node started - waiting for TF data...");
  }

private:
  void timer_callback()
  {
    // If we're still waiting for TF, check if the transforms are available
    if (wait_for_tf_) {
      try {
        // Check if we can find the map frame
        if (tf_buffer_->canTransform("map", "rick/base_link", tf2::TimePointZero, 500ms) &&
            tf_buffer_->canTransform("map", "morty/base_link", tf2::TimePointZero, 500ms)) {
          
          // Both transforms are available, we can start tracking
          wait_for_tf_ = false;
          tf_ready_ = true;
          RCLCPP_INFO(this->get_logger(), "TF data available! Waiting for Morty to move...");
          
          // Update timer to a more responsive rate now that we have TF
          timer_->cancel();
          timer_ = this->create_wall_timer(100ms, std::bind(&RobotChase::timer_callback, this));
          return;
        } 
        RCLCPP_INFO(this->get_logger(), "Waiting for TF data...");
        return;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF not ready: %s", ex.what());
        return;
      }
    }
    
    // If TF isn't ready, don't proceed
    if (!tf_ready_) {
      return;
    }
    
    // Always create a zero twist to publish in case of errors or if we don't need to move
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    
    try {
      // First try to get Morty's position in the map frame
      auto morty_transform = tf_buffer_->lookupTransform(
        "map", "morty/base_link", tf2::TimePointZero);
      
      // If we haven't initialized yet, store Morty's initial position
      if (!initialized_) {
        initial_morty_x_ = morty_transform.transform.translation.x;
        initial_morty_y_ = morty_transform.transform.translation.y;
        last_morty_x_ = initial_morty_x_;
        last_morty_y_ = initial_morty_y_;
        initialized_ = true;
        last_position_update_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), 
                  "Initialized - Morty's position: (%.2f, %.2f)",
                  initial_morty_x_, initial_morty_y_);
        publisher_->publish(std::move(twist_msg));  // Publish zero velocity
        return;
      }
      
      // Current Morty position
      double current_morty_x = morty_transform.transform.translation.x;
      double current_morty_y = morty_transform.transform.translation.y;
      
      // Calculate how far Morty has moved from initial position
      double morty_displacement = std::sqrt(
        std::pow(current_morty_x - initial_morty_x_, 2) + 
        std::pow(current_morty_y - initial_morty_y_, 2));
      
      // Get the current time for movement rate calculation
      auto current_time = this->now();
      double dt = (current_time - last_position_update_time_).seconds();
      
      // Check if Morty is moving - look at displacement from starting position
      if (morty_displacement > movement_threshold_) {
        if (!morty_is_moving_) {
          RCLCPP_INFO(this->get_logger(), "Morty is moving! Starting chase.");
          morty_is_moving_ = true;
        }
      }
      
      // Update last position
      last_morty_x_ = current_morty_x;
      last_morty_y_ = current_morty_y;
      last_position_update_time_ = current_time;
      
      // If Morty hasn't moved enough yet, don't chase
      if (!morty_is_moving_) {
        // Publish zero velocity and return
        publisher_->publish(std::move(twist_msg));
        return;
      }
      
      // If we're chasing, we need the transform from Rick to Morty
      auto rick_to_morty = tf_buffer_->lookupTransform(
        "rick/base_link", "morty/base_link", tf2::TimePointZero);
      
      // Extract the translation
      double x = rick_to_morty.transform.translation.x;
      double y = rick_to_morty.transform.translation.y;
      
      // Calculate the distance between Rick and Morty
      double error_distance = std::sqrt(x*x + y*y);
      
      // Calculate the yaw error (angle to target)
      double error_yaw = std::atan2(y, x);
      
      // Set linear velocity based on distance error
      if (error_distance > min_distance_) {
        // Linear speed is proportional to distance, but slower as we get closer
        twist_msg->linear.x = kp_distance_ * (error_distance - min_distance_);
      } else {
        // If we're close enough, stop moving forward
        twist_msg->linear.x = 0.0;
      }
      
      // Set angular velocity based on yaw error
      twist_msg->angular.z = kp_yaw_ * error_yaw;
      
      // Limit velocities for safety
      if (twist_msg->linear.x > max_linear_velocity_) {
        twist_msg->linear.x = max_linear_velocity_;
      }
      
      if (std::fabs(twist_msg->angular.z) > max_angular_velocity_) {
        twist_msg->angular.z = (twist_msg->angular.z > 0) ? 
                              max_angular_velocity_ : -max_angular_velocity_;
      }
      
      // Log information at a reasonable rate (not every callback)
      static int log_counter = 0;
      if (++log_counter % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), 
                    "Distance: %.2f, Angle: %.2f, Linear: %.2f, Angular: %.2f",
                    error_distance, error_yaw, twist_msg->linear.x, twist_msg->angular.z);
      }
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                           "Could not transform: %s", ex.what());
      // Don't return, still publish zero velocity below
    }
    
    // Publish the velocity command (will be zero if any errors occurred)
    publisher_->publish(std::move(twist_msg));
  }

  // Transform listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Publisher for velocity commands
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // PID gains
  double kp_distance_;
  double kp_yaw_;
  
  // Safety parameters
  double min_distance_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  
  // TF status
  bool wait_for_tf_;
  bool tf_ready_;
  
  // Movement tracking
  bool initialized_;
  bool morty_is_moving_;
  double initial_morty_x_;
  double initial_morty_y_;
  double last_morty_x_;
  double last_morty_y_;
  double movement_threshold_;
  rclcpp::Time last_position_update_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
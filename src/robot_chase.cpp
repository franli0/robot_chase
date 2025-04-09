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
    initial_distance_(0.0)
  {
    // Create a transform listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher for Rick's velocity commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

    // Create a timer to periodically check the transform and update velocity
    timer_ = this->create_wall_timer(
      100ms, std::bind(&RobotChase::timer_callback, this));

    // PID controller gains - these can be tuned for better performance
    kp_distance_ = 0.3;  // Proportional gain for distance
    kp_yaw_ = 1.0;       // Proportional gain for yaw
    
    // Safety parameters
    min_distance_ = 0.5;  // Minimum safe distance to maintain (increased for better behavior)
    max_linear_velocity_ = 0.5;  // Maximum linear velocity
    max_angular_velocity_ = 1.0;  // Maximum angular velocity
    
    // Movement detection parameters
    movement_threshold_ = 0.2;  // Distance Morty must move to trigger chase
    last_morty_x_ = 0.0;
    last_morty_y_ = 0.0;
    morty_movement_timer_ = 0;  // Counter to detect sustained movement
    
    RCLCPP_INFO(this->get_logger(), "Robot Chase node started - waiting for Morty to move");
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    
    // Try to get the transform between Rick's base_link and Morty's base_link
    try {
      // Get transform from map to morty/base_link to track Morty's absolute position
      auto morty_transform = tf_buffer_->lookupTransform(
        "map", "morty/base_link", tf2::TimePointZero);
        
      // Get transform from rick/base_link to morty/base_link for chasing calculations
      transform_stamped = tf_buffer_->lookupTransform(
        "rick/base_link", "morty/base_link", tf2::TimePointZero);
        
      // If we haven't initialized yet, store Morty's initial position
      if (!initialized_) {
        initial_morty_x_ = morty_transform.transform.translation.x;
        initial_morty_y_ = morty_transform.transform.translation.y;
        initialized_ = true;
        RCLCPP_INFO(this->get_logger(), 
                    "Initialized - Morty's position: (%.2f, %.2f)",
                    initial_morty_x_, initial_morty_y_);
        return;
      }
      
      // Current Morty position
      double current_morty_x = morty_transform.transform.translation.x;
      double current_morty_y = morty_transform.transform.translation.y;
      
      // Calculate how far Morty has moved from initial position
      double morty_displacement = std::sqrt(
        std::pow(current_morty_x - initial_morty_x_, 2) + 
        std::pow(current_morty_y - initial_morty_y_, 2));
        
      // Calculate how far Morty has moved since last check
      double morty_step_movement = std::sqrt(
        std::pow(current_morty_x - last_morty_x_, 2) + 
        std::pow(current_morty_y - last_morty_y_, 2));
        
      // Update last position
      last_morty_x_ = current_morty_x;
      last_morty_y_ = current_morty_y;
      
      // Check if Morty is moving (either from initial position or recently)
      if (morty_displacement > movement_threshold_) {
        // Morty has moved far enough from initial position
        if (!morty_is_moving_) {
          RCLCPP_INFO(this->get_logger(), "Morty is moving! Starting chase.");
          morty_is_moving_ = true;
        }
      }
      
      // If Morty hasn't moved enough yet, don't chase
      if (!morty_is_moving_) {
        // Don't move Rick yet
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        publisher_->publish(std::move(twist_msg));
        return;
      }
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // Extract the translation and rotation from the transform
    double x = transform_stamped.transform.translation.x;
    double y = transform_stamped.transform.translation.y;
    
    // Calculate the distance between Rick and Morty
    double error_distance = std::sqrt(x*x + y*y);
    
    // Calculate the yaw error (angle to target)
    double error_yaw = std::atan2(y, x);
    
    // Create Twist message for velocity
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    
    // Set linear velocity based on distance error, with a minimum distance threshold
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
    
    // Publish the velocity command
    publisher_->publish(std::move(twist_msg));
    
    RCLCPP_INFO(this->get_logger(), 
                "Distance: %.2f, Angle: %.2f, Linear: %.2f, Angular: %.2f",
                error_distance, error_yaw, twist_msg->linear.x, twist_msg->angular.z);
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
  
  // Movement tracking
  bool initialized_;
  bool morty_is_moving_;
  double initial_morty_x_;
  double initial_morty_y_;
  double last_morty_x_;
  double last_morty_y_;
  double initial_distance_;
  double movement_threshold_;
  int morty_movement_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotChase>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>


using std::placeholders::_1;
using namespace std::chrono_literals;



class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  ObstacleAvoidanceNode(double obstacle_distance, double degrees)
      : Node("obstacle_avoidance_node"), obstacle_distance_(obstacle_distance),
        degrees_(degrees), stop_rotation_(false), aligning_to_shelf_(false) {

    // Subscribe to the /scan topic
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ObstacleAvoidanceNode::scanCallback, this,
                  std::placeholders::_1));

    // Subscribe to the /odom topic to get the robot's yaw angle
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ObstacleAvoidanceNode::odometryCallback, this,
                  std::placeholders::_1));

    // Publish to the /robot/cmd_vel topic
    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

    // Set the linear velocity to move the robot forward
    linear_velocity_ =
        0.2; // You can adjust this value to your desired velocity

    // Create a timer to periodically check for obstacles and control the robot
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObstacleAvoidanceNode::timerCallback, this));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    float min_distance = *std::min_element(scan->ranges.begin(), scan->ranges.end());

    if (min_distance <= obstacle_distance_ && !stop_rotation_) {
      aligning_to_shelf_ = true;
      stop_rotation_ = true; // Stop rotating

      // Stop the robot and align to the shelf
      stopRobot();
      alignToShelf();
    } else if (aligning_to_shelf_ && std::abs(yaw_ - degrees_) < 0.05) {
      // Continue aligning to the shelf until the desired angle is reached
      stopRobot();
      alignToShelf();
    } else if (!stop_rotation_) {
      // If the robot is not already stopped and not aligning to the shelf, move
      // it forward
      moveRobotForward();
    }
}


  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    // Process odometry data quickly
    // Avoid time-consuming operations here

    // Get the robot's yaw angle from the odometry data
    auto orientation = odom->pose.pose.orientation;
    // Convert the orientation to a quaternion and extract the yaw from the
    // quaternion
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;
    
  }

  void timerCallback() {
    // This function is called periodically by the timer.
    // You can perform more complex or time-consuming operations here if needed.
    // This function runs in a separate timer thread.
  }

  void moveRobotForward() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_velocity_;
    twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(twist);
  }

  void stopRobot() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_publisher_->publish(twist);
  }

  void rotateRobot() {
    auto twist = geometry_msgs::msg::Twist();

    double angle_difference = std::abs(yaw_ - degrees_);

    if (angle_difference <= 0.05) {
      // If the desired angle is reached (within a tolerance), stop rotating
      twist.angular.z = 0.0;
      stop_rotation_ = false;
    } else {
      // Continue rotating towards the desired angle in radians
      twist.angular.z = degrees_ > 0 ? 0.2 : -0.2;
    }

    cmd_vel_publisher_->publish(twist);
  }

  void alignToShelf() {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;

    double angle_difference = std::abs(yaw_ - degrees_);

    if (angle_difference <= 0.05) {
        // If the desired angle is reached (within a tolerance), stop rotation
        twist.angular.z = 0.0;
        aligning_to_shelf_ = false; // Added this line
    } else {
        // Calculate the rotation direction (positive or negative)
        double rotation_direction = (yaw_ < degrees_) ? 1.0 : -1.0;
        twist.angular.z = rotation_direction * 0.2; // Use the calculated direction
    }

    cmd_vel_publisher_->publish(twist);
}

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  float linear_velocity_;
  float obstacle_distance_;
  double degrees_; // Degrees converted to radians
  bool stop_rotation_;
  bool aligning_to_shelf_;
  double yaw_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  double obstacle_distance = 0.3; // Default value
  double degrees = -90.0;         // Default value in degrees

  // Convert degrees to radians
  double radians = degrees * M_PI / 180.0;

  // Create a ROS 2 Node with a private namespace
  auto node = std::make_shared<rclcpp::Node>("obstacle_avoidance_node");

  // Get parameter values from the command-line arguments
  if (node->has_parameter("obstacle")) {
    node->get_parameter("obstacle", obstacle_distance);
  }
  if (node->has_parameter("degrees")) {
    node->get_parameter("degrees", degrees);
    radians = degrees * M_PI / 180.0; // Convert degrees to radians
  }

  // Create the ObstacleAvoidanceNode with parameter values
  auto obstacle_avoidance_node =
      std::make_shared<ObstacleAvoidanceNode>(obstacle_distance, radians);

  // Spin the node
  rclcpp::spin(obstacle_avoidance_node);

  rclcpp::shutdown();
  return 0;
}

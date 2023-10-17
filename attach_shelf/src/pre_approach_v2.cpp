#include "checkpoint5_interfaces/srv/detail/go_to_loading__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "checkpoint5_interfaces/srv/go_to_loading.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <mutex>

using std::placeholders::_1;
using namespace std::chrono_literals;

class ObstacleAvoidanceNode : public rclcpp::Node {
public:
  ObstacleAvoidanceNode()
      : Node("obstacle_avoidance_node") {

    float obstacle_distance_ = 0.0;
    double degrees_ = 0.0;

    this->declare_parameter<double>("obstacle_distance", 0.3); // Default value of 0.3 meters
    this->declare_parameter<double>("degrees", -90.0); // Default value of -90 degrees

    // Then, get the parameters' values
    this->get_parameter("obstacle_distance", obstacle_distance_);
    this->get_parameter("degrees", degrees_);
    degrees_ = degrees_ * M_PI / 180.0; // Convert degrees to radians





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
        std::bind(&ObstacleAvoidanceNode::callService, this));

    //create a client to call the checkpoint5_interface::srv::GoToLoading service
    approach_shelf_client_ = this->create_client<checkpoint5_interfaces::srv::GoToLoading>("/approach_shelf");
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


  void callService() {
    
    //create a request
    auto request = std::make_shared<checkpoint5_interfaces::srv::GoToLoading::Request>();
    //set the request to attach
    request->attach_to_shelf = true;
    while(!approach_shelf_client_->wait_for_service(std::chrono::seconds(1))){
        if(!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_future = approach_shelf_client_->async_send_request(request, std::bind(&ObstacleAvoidanceNode::response_callback, this, std::placeholders::_1));
  
  }

  void response_callback(
      rclcpp::Client<checkpoint5_interfaces::srv::GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      service_response_ = future.get()->complete;
      RCLCPP_INFO(this->get_logger(), "Result: success: %d", service_response_);
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
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
    //call service
    callService();
}

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Client<checkpoint5_interfaces::srv::GoToLoading>::SharedPtr approach_shelf_client_;
  //create a memeber for

  //create a memeber for service response
  float linear_velocity_;
  float obstacle_distance_;
  double degrees_; // Degrees converted to radians
  bool stop_rotation_;
  bool aligning_to_shelf_;
  double yaw_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_response_ = false;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
// Create the ObstacleAvoidanceNode with parameter values
  auto obstacle_avoidance_node = std::make_shared<ObstacleAvoidanceNode>();

  // Spin the node
  rclcpp::spin(obstacle_avoidance_node);

  rclcpp::shutdown();
  return 0;
}

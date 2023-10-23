#include "geometry_msgs/msg/twist.hpp"
#include "checkpoint5_interfaces/srv/detail/go_to_loading__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "checkpoint5_interfaces/srv/go_to_loading.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

struct Point {
    double x;
    double y;
};

struct Leg {
    int startIndex;
    int endIndex;
    int middleIndex;
    Point position;
};

//create serviceServer class
class serviceServer : public rclcpp::Node
{
    public:
    serviceServer() : Node("service_server")
    {
        
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&serviceServer::laserCallback, this, _1));
        srv_ = create_service<checkpoint5_interfaces::srv::GoToLoading>("attach_shelf", std::bind(&serviceServer::handleService, this, std::placeholders::_1, std::placeholders::_2));
        //create a publisher for the velocity
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        //create a subscriber for the odometry
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    std::pair<double, double> getRobotCurrentPosition()
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        return {current_position_x_, current_position_y_};
    }

    private:

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) 
    {
        // store the laser message
        laser_msg_ = scan;
        // store the laser intensities
        intensities_ = scan->intensities;
        // store the laser ranges
        ranges_ = scan->ranges;
    }
    void setupOdomSubscriber()
    {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&serviceServer::odomCallback, this, std::placeholders::_1));
    }
    
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_position_x_ = msg->pose.pose.position.x;
        current_position_y_ = msg->pose.pose.position.y;
    }

void detectLeg()
{

    //loop through the intensities
    bool insideLeg = false;
    int currentStartIndex = -1;

    for (std::size_t i = 0; i < intensities_.size(); ++i) {
        if (intensities_[i] == 8000 && (i == 0 || intensities_[i - 1] != 8000)) {
            insideLeg = true;
            currentStartIndex = i;
            //display the start of the leg to the terminal via RCLCPP_INFO
            RCLCPP_INFO(this->get_logger(), "Detected start of leg at index %zu", i);

        } else if (insideLeg && intensities_[i] != 8000) {
            int middle = (currentStartIndex + i - 1) / 2;
            //display the middle of the leg to the terminal via RCLCPP_INFO
            RCLCPP_INFO(this->get_logger(), "Detected middle of leg at index %d", middle);
            legs.push_back({currentStartIndex, static_cast<int>(i - 1), middle, {0.0, 0.0}});
            insideLeg = false;
            //display the end of the leg to the terminal via RCLCPP_INFO
            RCLCPP_INFO(this->get_logger(), "Detected end of leg at index %zu", (i - 1));

        }
    }

    if (insideLeg) {
        int middle = (currentStartIndex + intensities_.size() - 1) / 2;
        legs.push_back({currentStartIndex, static_cast<int>(intensities_.size()) - 1, middle, {0.0, 0.0}});
    }
    
    
}

// Function to broadcast a transform for the cart_frame
void broadcastTransform(Point center) 
{
    static tf2_ros::StaticTransformBroadcaster broadcaster(this);
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "/robot/base_link"; 
    transformStamped.child_frame_id = "cart_frame";
    transformStamped.transform.translation.x = center.x;
    transformStamped.transform.translation.y = center.y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);  // Assuming no rotation between the frames
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    broadcaster.sendTransform(transformStamped);
}

void approachCartFrame()
{   

    // Loop to keep checking the transform and adjust the robot's position
    while (rclcpp::ok())
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform("robot_base_link", "cart_frame", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            rclcpp::sleep_for(100ms);
            continue;
        }

        // Compute error
        double error_x = transformStamped.transform.translation.x;
        double error_y = transformStamped.transform.translation.y;
        double error_distance = std::sqrt(error_x * error_x + error_y * error_y);
        
        // If the robot is close enough to the cart_frame, stop and move forward 30cm
        if (error_distance < 0.05)  // 5cm threshold
        {
            // the robot moves forward 30cm.
            moveForwardBy30cm();
            break;
        }
        
        double error_angle = std::atan2(error_y, error_x);

        // Calculate control actions
        double linear_velocity = kp_linear * error_distance;
        double angular_velocity = kp_angular * error_angle;

        // Publish velocities
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        
        //  publisher to send cmd_vel
        velocity_publisher_->publish(cmd_vel);

        rclcpp::sleep_for(10ms);
    }
}

void moveForwardBy30cm()
{
    // This assumes you have access to the robot's current position
    // which you'd typically get via odometry or another localization mechanism
    setupOdomSubscriber();
    std::pair<double, double> current_position = getRobotCurrentPosition();
    double starting_position_x = current_position_x_;
    double final_position_x = starting_position_x + desired_distance;

    while (rclcpp::ok())
    {
        current_position = getRobotCurrentPosition();
        double current_position_x = current_position.first;

        // Compute error
        double error = final_position_x - current_position_x;

        // Check if the robot is close enough to the desired position
        if (std::abs(error) < threshold)
        {
            // Stop the robot
            geometry_msgs::msg::Twist cmd_vel_stop;
            cmd_vel_stop.linear.x = 0.0;
            cmd_vel_stop.angular.z = 0.0;
            // TODO: Use your velocity publisher to send cmd_vel_stop
             velocity_publisher_->publish(cmd_vel_stop);
            break;
        }

        // Calculate control action
        double linear_velocity = kp_forward * error;

        // Publish velocities
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = 0.0;  // No angular movement required
        
        // velocity publisher to send cmd_vel
        velocity_publisher_->publish(cmd_vel);

        rclcpp::sleep_for(10ms); 
    }
}


void handleService(const std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Request> request,
      const std::shared_ptr<checkpoint5_interfaces::srv::GoToLoading::Response> response)
    {

        RCLCPP_INFO(this->get_logger(), "Requested /attach_shelf Service: %d", request->attach_to_shelf);
        if (request->attach_to_shelf == true){

        detectLeg();
        // Checking if less than 2 legs are detected
        if(legs.size() < 2) {
            RCLCPP_INFO(this->get_logger(), "Two legs NOT detected: ");
            response->complete = false;
            return;   
        
        }

        // Calculate center point between the two legs
        Point center;
        center.x = (legs[0].position.x + legs[1].position.x) / 2.0;
        center.y = (legs[0].position.y + legs[1].position.y) / 2.0;

        // Broadcast transform for the cart frame at the center point
        broadcastTransform(center);

        // If attach_to_shelf is True, then perform the final approach
        if (request->attach_to_shelf) {
        // Make the robot move towards the center using the transform 
        approachCartFrame();
        
        // TODO: Implement logic to lift the robot/shelf

        response->complete = true;
         } else {
        // If attach_to_shelf is False, only publish the transform
        response->complete = true; // Set to true since the action (publishing transform) was completed successfully
    }
        }
      
}

 
    //create a subscriber for the laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Service<checkpoint5_interfaces::srv::GoToLoading>::SharedPtr srv_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    sensor_msgs::msg::LaserScan::SharedPtr laser_msg_;

     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
     std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};


    //create variable for the laser intensities
    std::vector<float> intensities_;
    //create variable for the laser ranges
    std::vector<float> ranges_;
    //vector for legs
    std::vector<Leg> legs;

    std::mutex odom_mutex_;  
    
    //for move forward by 30cm function////////////
    double desired_distance = 0.30; // 30cm
    double threshold = 0.02; // 2cm threshold to stop
    double kp_forward = 0.5; // Proportional gain for forward movement
    /////////////////////////////
    //for approach cart frame function
    double kp_linear = 0.5;  // Proportional gain for linear velocity
    double kp_angular = 1.0;  // Proportional gain for angular velocity
    /////////////////////////////
    double current_position_x_ = 0.0;
    double current_position_y_ = 0.0;

};

//main function
int main(int argc, char **argv)
{
    //initialize ros
    rclcpp::init(argc, argv);
    //create a node
    auto node = std::make_shared<serviceServer>();
    //spin the node
    rclcpp::spin(node);
    //shutdown ros
    rclcpp::shutdown();
    return 0;
}
